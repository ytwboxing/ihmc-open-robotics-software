package us.ihmc.humanoidBehaviors.lookAndStep;

import com.google.common.collect.Lists;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorState.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   public enum LookAndStepBehaviorState
   {
      PERCEPT, PLAN, USER, STEP
   }

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private final StateMachine<LookAndStepBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;
   private final RemoteREAInterface rea;

   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;

   private AtomicReference<FootstepPlannerOutput> latestFootstepPlannerOutput = new AtomicReference<>();
   private final TypedNotification<FootstepPlannerOutput> footstepPlannerOutputNotification = new TypedNotification<>();
   private final Notification takeStepNotification;
   private final Notification rePlanNotification;

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      footPolygons = helper.createFootPolygons();
      rea = helper.getOrCreateREAInterface();
      robot = helper.getOrCreateRobotInterface();
      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      rePlanNotification = helper.createUINotification(RePlan);
      takeStepNotification = helper.createUINotification(TakeStep);
      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      walkingControllerParameters = helper.getRobotModel().getWalkingControllerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);
      helper.createUICallback(TakeStep, clicked -> takeStepNotification.set());

      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());

      EnumBasedStateMachineFactory<LookAndStepBehaviorState> stateMachineFactory = new EnumBasedStateMachineFactory<>(LookAndStepBehaviorState.class);
      stateMachineFactory.setDoAction(PERCEPT, this::doPerceptStateAction);
      stateMachineFactory.addTransition(PERCEPT, PLAN, this::transitionFromPercept);
      stateMachineFactory.setOnEntry(PLAN, this::onPlanStateEntry);
      stateMachineFactory.setDoAction(PLAN, this::doPlanStateAction);
      stateMachineFactory.addTransition(PLAN, Lists.newArrayList(USER, PERCEPT, STEP), this::transitionFromPlan);
      stateMachineFactory.setDoAction(USER, this::doUserStateAction);
      stateMachineFactory.addTransition(USER, Lists.newArrayList(STEP, PERCEPT), this::transitionFromUser);
      stateMachineFactory.setOnEntry(STEP, this::onStepStateEntry);
      stateMachineFactory.setDoAction(STEP, this::doStepStateAction);
      stateMachineFactory.addTransition(STEP, PERCEPT, this::transitionFromStep);
      stateMachineFactory.getFactory().addStateChangedListener(this::stateChanged);
      stateMachine = stateMachineFactory.getFactory().build(PERCEPT);

      mainThread = helper.createPausablePeriodicThread(getClass(), 0.1, this::lookAndStepThread);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Look and step behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void lookAndStepThread()   // update the active state
   {
      stateMachine.doActionAndTransition();
   }

   private void stateChanged(LookAndStepBehaviorState from, LookAndStepBehaviorState to)
   {
      helper.publishToUI(CurrentState, to.name());
      LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
   }

   private void doPerceptStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private boolean transitionFromPercept(double timeInState)
   {
      return !arePlanarRegionsExpired();
   }

   private void onPlanStateEntry()
   {
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      PlanarRegionsList latestPlanarRegionList = rea.getLatestPlanarRegionsList();
      helper.publishToUI(MapRegionsForUI, latestPlanarRegionList);

      FramePose3D goalPoseBetweenFeet = new FramePose3D();
      goalPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      goalPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = goalPoseBetweenFeet.getZ();

      goalPoseBetweenFeet.setToZero(latestHumanoidRobotState.getPelvisFrame());
      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.stepLength), 0.0, 0.0);
      goalPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      goalPoseBetweenFeet.setZ(midFeetZ);

      RobotSide initialStanceFootSide = null;
      FramePose3D leftSolePose = new FramePose3D();
      leftSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.LEFT));
      leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose3D rightSolePose = new FramePose3D();
      rightSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.RIGHT));
      rightSolePose.changeFrame(ReferenceFrame.getWorldFrame());

      if (leftSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()) <= rightSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()))
      {
         initialStanceFootSide = RobotSide.LEFT;
      }
      else
      {
         initialStanceFootSide = RobotSide.RIGHT;
      }

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();

      footstepPlannerRequest.setInitialStanceSide(initialStanceFootSide);
      footstepPlannerRequest.setPlanarRegionsList(latestPlanarRegionList);
      footstepPlannerRequest.setGoalPose(goalPoseBetweenFeet);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);

      footstepPlanningModule.addStatusCallback(status -> LogTools.info("Planning steps: {}", status.getFootstepPlan().getNumberOfSteps()));

      ThreadTools.startAsDaemon(() -> footstepPlanningThread(footstepPlannerRequest), "FootstepPlanner");
   }

   private void footstepPlanningThread(FootstepPlannerRequest footstepPlannerRequest)
   {
      footstepPlannerOutputNotification.add(footstepPlanningModule.handleRequest(footstepPlannerRequest));
   }

   private void doPlanStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private LookAndStepBehaviorState transitionFromPlan(double timeInState)
   {
      if (footstepPlannerOutputNotification.hasNext())
      {
         if (footstepPlannerOutputNotification.peek().getResult().validForExecution())
         {
            if (operatorReviewEnabledInput.get())
            {
               return USER;
            }
            else
            {
               return STEP;
            }
         }
         else
         {
            return PERCEPT;
         }
      }

      return null;
   }

   private void doUserStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private LookAndStepBehaviorState transitionFromUser(double timeInState)
   {
      // expired and planning enabled? percept
      // user clicked, valid?
      if (arePlanarRegionsExpired() && operatorReviewEnabledInput.get())
      {
         return PERCEPT;
      }
      else if (takeStepNotification.read())
      {
         return USER;
      }

      return null;
   }

   private void onStepStateEntry()
   {
            FootstepPlan shortenedFootstepPlan = new FootstepPlan();
            if (footstepPlan.getNumberOfSteps() > 0)
            {
               shortenedFootstepPlan.addFootstep(footstepPlan.getFootstep(0));
            }

      // send footstep plan to UI
      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlan));

      if (takeStep.poll())
      {
         // take step

         LogTools.info("Requesting walk");
         TypedNotification<WalkingStatusMessage> walkingStatusNotification = robot.requestWalk(FootstepDataMessageConverter.createFootstepDataListFromPlan(
               shortenedFootstepPlan,
               1.0,
               0.5,
               ExecutionMode.OVERRIDE), robot.pollHumanoidRobotState(), true, latestPlanarRegionList);

         walkingStatusNotification.blockingPoll();
      }
   }

   private void doStepStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private boolean transitionFromStep(double timeInState)
   {
      return !robot.isRobotWalking();
   }

   private void pollInterrupts()
   {
      footstepPlannerOutputNotification.poll();
      takeStepNotification.poll();
   }

   private boolean arePlanarRegionsExpired()
   {
      return rea.getPlanarRegionsListExpired(lookAndStepParameters.getPlanarRegionsExpiration());
   }

   public static class LookAndStepBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
      private static final CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

      public static final Topic<String> CurrentState = topic("CurrentState");
      public static final Topic<Object> TakeStep = topic("TakeStep");
      public static final Topic<Object> RePlan = topic("RePlan");
      public static final Topic<Boolean> OperatorReviewEnabled = topic("OperatorReview");
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> FootstepPlanForUI = topic("FootstepPlan");
      public static final Topic<PlanarRegionsList> MapRegionsForUI = topic("MapRegionsForUI");
      public static final Topic<List<String>> LookAndStepParameters = topic("LookAndStepParameters");
      public static final Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");

      private static final <T> Topic<T> topic(String name)
      {
         return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static final MessagerAPIFactory.MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}