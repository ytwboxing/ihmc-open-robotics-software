package us.ihmc.humanoidBehaviors.patrol;

import boofcv.core.image.FactoryGImageMultiBand.PL;
import com.google.common.collect.Lists;
import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState.*;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior
{
   private final StateMachine<PatrolBehaviorState, State> stateMachine;

   enum PatrolBehaviorState
   {
      /** Stop state that waits for or is triggered by a GoToWaypoint message */
      STOP,
      /** Request and wait for footstep planner result */
      PLAN,
      /** Walking towards goal waypoint */
      WALK
   }

   private final Messager messager;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;
   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   private final Notification stopNotification = new Notification();
   private final Notification overrideGoToWaypointNotification = new Notification();

   private final AtomicInteger goalWaypointIndex = new AtomicInteger();

   private TypedNotification<FootstepPlanningToolboxOutputStatus> footstepPlanResultNotification;
   private final Notification walkingCompleted = new Notification();

   private final AtomicReference<ArrayList<Pose3D>> waypoints;

   public PatrolBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      this.messager = messager;

      LogTools.debug("Initializing patrol behavior");

      CustomBehaviorStateMachineFactory<PatrolBehaviorState> factory = new CustomBehaviorStateMachineFactory<>(PatrolBehaviorState.class);
      factory.getStateMap().get(STOP).setOnEntry(this::onStopStateEntry);
      factory.getStateMap().get(STOP).setDoAction(this::doStopStateAction);
      factory.getFactory().addTransition(STOP, PLAN, this::transitionFromStopToPlan);
      factory.getStateMap().get(PLAN).setOnEntry(this::onPlanStateEntry);
      factory.getStateMap().get(PLAN).setDoAction(this::doPlanStateAction);
      factory.addTransition(PLAN, this::transitionFromPlan);
      factory.getStateMap().get(WALK).setOnEntry(this::onWalkStateEntry);
      factory.getStateMap().get(WALK).setDoAction(this::doWalkStateAction);
      factory.getStateMap().get(WALK).setOnExit(this::onWalkStateExit);
      factory.addTransition(WALK, this::transitionFromWalk);
      stateMachine = factory.getFactory().build(STOP);

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      pausePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(PauseWalkingCommand.class).getMessageClass(),
                                                 ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      new ROS2Callback<>(ros2Node, WalkingStatusMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.HUMANOID_CONTROL_MODULE, this::acceptWalkingStatus);

      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.REA_MODULE);

      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel);

      messager.registerTopicListener(API.Stop, object -> stopNotification.set());
      messager.registerTopicListener(API.GoToWaypoint, goToWaypointIndex -> {
         goalWaypointIndex.set(goToWaypointIndex);
         LogTools.info("Interrupted with GO_TO_WAYPOINT {}", goalWaypointIndex.get());
         overrideGoToWaypointNotification.set();
      });

      waypoints = messager.createInput(API.Waypoints);

      PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      patrolThread.schedule(this::patrolThread, 750, TimeUnit.MILLISECONDS); // TODO tune this up, 500Hz is probably too much
   }

   private void patrolThread()   // pretty much just updating whichever state is active
   {
      LogTools.debug("Current state: {}", stateMachine.getCurrentStateKey());
      stateMachine.doActionAndTransition();
   }

   private void onStopStateEntry()
   {
      LogTools.debug("onStopStateEntry");
      messager.submitMessage(API.CurrentState, STOP.name());

      sendPauseWalking(); // TODO make into tool
   }

   private void doStopStateAction(double timeInState)
   {
      LogTools.debug("doStopStateAction");
      pollInterrupts();
   }

   private boolean transitionFromStopToPlan(double timeInState)
   {
      LogTools.debug("Checking transition from STOP");

      boolean transition = overrideGoToWaypointNotification.read() && goalWaypointInBounds();
      if (transition)
      {
         LogTools.debug("STOP -> PLAN (overrideGoToWaypointNotification + goalWaypointInBounds)");
      }
      return transition;
   }

   private void onPlanStateEntry()
   {
      LogTools.debug("onPlanStateEntry");
      messager.submitMessage(API.CurrentState, PLAN.name());

      remoteFootstepPlannerInterface.abortPlanning();

      FramePose3D midFeetZUpPose = new FramePose3D();
      // prevent frame from continuing to change
      midFeetZUpPose.setFromReferenceFrame(remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames().getMidFeetZUpFrame());
      int index = goalWaypointIndex.get();
      messager.submitMessage(API.CurrentWaypointIndexStatus, index);
      FramePose3D currentGoalWaypoint = new FramePose3D(waypoints.get().get(index));

      footstepPlanResultNotification = remoteFootstepPlannerInterface.requestPlan(midFeetZUpPose, currentGoalWaypoint, planarRegionsList.getLatest());
   }

   private void doPlanStateAction(double timeInState)
   {
      LogTools.debug("doPlanStateAction");
      pollInterrupts();
      footstepPlanResultNotification.poll();
   }

   private PatrolBehaviorState transitionFromPlan(double timeInState)
   {
      LogTools.debug("Checking transition from PLAN");
      if (stopNotification.read())
      {
         LogTools.debug("PLAN -> STOP (stopNotification)");
         return STOP;
      }
      else if (overrideGoToWaypointNotification.read())
      {
         if (goalWaypointInBounds())
         {
            LogTools.debug("PLAN -> PLAN (overrideGoToWaypointNotification + goalWaypointInBounds)");
            return PLAN;
         }
         else
         {
            LogTools.debug("PLAN -> STOP (overrideGoToWaypointNotification + !goalWaypointInBounds)");
            return STOP;
         }
      }
      else if (footstepPlanResultNotification.hasNext())
      {
         if (FootstepPlanningResult.fromByte(footstepPlanResultNotification.read().getFootstepPlanningResult()).validForExecution())
         {
            LogTools.debug("PLAN -> WALK (footstepPlanResultNotification + validForExecution)");
            return WALK;
         }
         else
         {
            LogTools.debug("PLAN -> PLAN (footstepPlanResultNotification + !validForExecution)");
            return PLAN;
         }
      }

      return null;
   }

   private void onWalkStateEntry()
   {
      LogTools.debug("onWalkStateEntry");
      messager.submitMessage(API.CurrentState, WALK.name());

      FootstepPlanningToolboxOutputStatus footstepPlanningOutput = footstepPlanResultNotification.read();
      FootstepPlan footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(footstepPlanningOutput.getFootstepDataList());
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      messager.submitMessage(API.CurrentFootstepPlan, footstepLocations);

      walkingCompleted.poll(); // acting to clear the notification TODO return walk result same as footstep plan
      LogTools.debug("Tasking {} footstep(s) to the robot", footstepPlanningOutput.getFootstepDataList().getFootstepDataList().size());
      footstepDataListPublisher.publish(footstepPlanningOutput.getFootstepDataList());
   }

   private void doWalkStateAction(double timeInState)
   {
      LogTools.debug("doWalkStateAction");
      pollInterrupts();
      walkingCompleted.poll();
   }

   private PatrolBehaviorState transitionFromWalk(double timeInState)
   {
      LogTools.debug("Checking transition from WALK");
      if (stopNotification.read())
      {
         LogTools.debug("WALK -> STOP (stopNotification)");
         return STOP;
      }
      else if (overrideGoToWaypointNotification.read())
      {
         if (goalWaypointInBounds())
         {
            LogTools.debug("WALK -> PLAN (overrideGoToWaypointNotification + goalWaypointInBounds)");
            return PLAN;
         }
         else
         {
            LogTools.debug("WALK -> PLAN (overrideGoToWaypointNotification + !goalWaypointInBounds)");
            return STOP;
         }
      }
      else if (walkingCompleted.read())
      {
         LogTools.debug("WALK -> PLAN (walkingCompleted)");
         return PLAN;
      }

      return null;
   }

   private void onWalkStateExit()
   {
      LogTools.debug("onWalkStateExit");
      if (!stopNotification.read() && !overrideGoToWaypointNotification.read()) // only increment if WALK -> PLAN
      {
         ArrayList<Pose3D> latestWaypoints = waypoints.get();      // access and store these early
         int nextGoalWaypointIndex = goalWaypointIndex.get() + 1;  // to make thread-safe
         if (nextGoalWaypointIndex >= latestWaypoints.size())
            nextGoalWaypointIndex = 0;
         goalWaypointIndex.set(nextGoalWaypointIndex);
      }
   }

   private void pollInterrupts()
   {
      stopNotification.poll();         // poll both at the same time to handle race condition
      overrideGoToWaypointNotification.poll();
   }

   private void sendPauseWalking()
   {
      LogTools.debug("Sending pause walking to robot");
      PauseWalkingMessage pause = new PauseWalkingMessage();
      pause.setPause(true);
      pausePublisher.publish(pause);
   }

   private boolean goalWaypointInBounds()
   {
      ArrayList<Pose3D> latestWaypoints = waypoints.get();     // access and store these early
      int currentGoalWaypointIndex = goalWaypointIndex.get();  // to make thread-safe
      boolean indexInBounds = currentGoalWaypointIndex >= 0 && currentGoalWaypointIndex < latestWaypoints.size();
      return indexInBounds;
   }

   private void acceptWalkingStatus(WalkingStatusMessage message)
   {
      LogTools.debug("Walking status: {}", WalkingStatus.fromByte(message.getWalkingStatus()).name());
      if (message.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         walkingCompleted.set();
      }
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("PatrolBehavior");
      private static final CategoryTheme Patrol = apiFactory.createCategoryTheme("Patrol");

      /** Input: Update the waypoints */
      public static final Topic<ArrayList<Pose3D>> Waypoints = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Waypoints"));

      /** Input: Robot stops and immediately goes to this waypoint. The "start" or "reset" command.  */
      public static final Topic<Integer> GoToWaypoint = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("GoToWaypoint"));

      /** Input: When received, the robot stops walking and waits forever. */
      public static final Topic<Object> Stop = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("Stop"));

      /** Output: to visualize the current robot path plan. */
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> CurrentFootstepPlan = Root.child(Patrol)
                                                                                              .topic(apiFactory.createTypedTopicTheme("CurrentFootstepPlan"));

      /** Output: to visualize the current state. */
      public static final Topic<String> CurrentState = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentState"));

      /** Output: to visualize the current waypoint status. TODO clean me up */
      public static final Topic<Integer> CurrentWaypointIndexStatus = Root.child(Patrol).topic(apiFactory.createTypedTopicTheme("CurrentWaypointIndexStatus"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
