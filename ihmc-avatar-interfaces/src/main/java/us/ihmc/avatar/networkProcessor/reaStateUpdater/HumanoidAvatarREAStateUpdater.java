package us.ihmc.avatar.networkProcessor.reaStateUpdater;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import std_msgs.msg.dds.Bool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class HumanoidAvatarREAStateUpdater implements CloseableAndDisposable
{
   private static final ROS2Topic<Bool> ENABLE_BASE = ROS2Tools.IHMC_ROOT.withType(Bool.class).withInput().withSuffix("enable");

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   private final REAStateRequestMessage clearRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage pauseRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage resumeRequestMessage = new REAStateRequestMessage();
   private final REAStateRequestMessage clearAndResumeRequestMessage = new REAStateRequestMessage();

   private boolean enabled = true;

   public static ROS2Topic<Bool> getEnableTopic(String specifier)
   {
      return ENABLE_BASE.withModule("rea_updater_" + specifier);
   }

   public HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel, PubSubImplementation implementation)
   {
      this(robotModel, implementation, "");
   }

   public HumanoidAvatarREAStateUpdater(DRCRobotModel robotModel, PubSubImplementation implementation, String specifier)
   {
      String robotName = robotModel.getSimpleRobotName();

      clearRequestMessage.setRequestClear(true);
      pauseRequestMessage.setRequestPause(true);
      resumeRequestMessage.setRequestResume(true);
      clearAndResumeRequestMessage.setRequestClear(true);
      clearAndResumeRequestMessage.setRequestResume(true);

      ros2Node = ROS2Tools.createROS2Node(implementation, "avatar_rea_state_updater");

      reaStateRequestPublisher = ROS2Tools.createPublisher(ros2Node, REACommunicationProperties.stateRequest.withSuffix(specifier));
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ControllerAPIDefinition.getTopic(HighLevelStateChangeStatusMessage.class, robotName),
                                           this::handleHighLevelStateChangeMessage);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotName),
                                           this::handleWalkingStatusMessage);
      ROS2Tools.createCallback(ros2Node, getEnableTopic(specifier), enable -> enabled = enable.getData());
   }

   private void handleHighLevelStateChangeMessage(Subscriber<HighLevelStateChangeStatusMessage> subscriber)
   {
      if (executorService.isShutdown() || !enabled)
         return;

      HighLevelStateChangeStatusMessage newMessage = subscriber.takeNextData();

      if (newMessage.getInitialHighLevelControllerName() == newMessage.getEndHighLevelControllerName())
         return;

      switch (HighLevelControllerName.fromByte(newMessage.getEndHighLevelControllerName()))
      {
      case WALKING:
         executorService.execute(() -> reaStateRequestPublisher.publish(clearAndResumeRequestMessage));
         break;
      default:
         executorService.execute(() -> reaStateRequestPublisher.publish(pauseRequestMessage));
         break;
      }
   }

   private void handleWalkingStatusMessage(Subscriber<WalkingStatusMessage> subscriber)
   {
      if (executorService.isShutdown() || !enabled)
         return;

      WalkingStatusMessage newMessage = subscriber.takeNextData();

      switch (WalkingStatus.fromByte(newMessage.getWalkingStatus()))
      {
      case STARTED:
      case RESUMED:
         executorService.execute(() -> reaStateRequestPublisher.publish(pauseRequestMessage));
         break;
      case COMPLETED:
      case PAUSED:
         executorService.execute(() -> reaStateRequestPublisher.publish(resumeRequestMessage));
         break;
      case ABORT_REQUESTED:
      default:
         // Do nothing?
         break;
      }
   }

   private void shutdown()
   {
      executorService.shutdownNow();
      ros2Node.destroy();
   }

   @Override
   public void closeAndDispose()
   {
      shutdown();
   }
}
