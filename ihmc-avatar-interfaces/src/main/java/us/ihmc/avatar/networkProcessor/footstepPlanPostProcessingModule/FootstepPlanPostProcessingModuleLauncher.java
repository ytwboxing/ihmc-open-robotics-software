package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import controller_msgs.msg.dds.FootstepPostProcessingParametersPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class FootstepPlanPostProcessingModuleLauncher
{
   public static FootstepPlanPostProcessingModule createModule(DRCRobotModel robotModel)
   {
      String moduleName = robotModel.getSimpleRobotName();
      FootstepPostProcessingParametersBasics parameters = robotModel.getFootstepPostProcessingParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ICPPlannerParameters cmpPlannerParameters = robotModel.getCapturePointPlannerParameters();

      return new FootstepPlanPostProcessingModule(moduleName, parameters, walkingControllerParameters, contactPointParameters, cmpPlannerParameters);
   }

   public static FootstepPlanPostProcessingModule createModule(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "footstep_post_processor");

      return createModule(ros2Node, robotModel);
   }

   public static FootstepPlanPostProcessingModule createModule(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      FootstepPlanPostProcessingModule postProcessingModule = createModule(robotModel);

      postProcessingModule.registerRosNode(ros2Node);
      String name = postProcessingModule.getName();

      ROS2Topic inputTopic = ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX.withRobot(name)
                                                                          .withInput();
      ROS2Topic outputTopic = ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX.withRobot(name)
                                                                           .withOutput();

      // Parameters callback
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPostProcessingParametersPacket.class, inputTopic,
                                           s -> postProcessingModule.getParameters().set(s.readNextData()));

      // Planner request callback
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPostProcessingPacket.class, inputTopic, s -> {
         FootstepPostProcessingPacket requestPacket = s.takeNextData();
         new Thread(() -> postProcessingModule.handleRequestPacket(requestPacket)).start();
      });

      // Result publisher
      IHMCROS2Publisher<FootstepPostProcessingPacket> resultPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, FootstepPostProcessingPacket.class, outputTopic);
      postProcessingModule.addStatusCallback(resultPublisher::publish);

      return postProcessingModule;
   }

}