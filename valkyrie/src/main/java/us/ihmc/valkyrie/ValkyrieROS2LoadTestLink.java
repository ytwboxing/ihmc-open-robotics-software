package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

public class ValkyrieROS2LoadTestLink
{
   public static final int NUMBER_OF_PUBS = 20;
   public static final int NUMBER_OF_SUBS = ValkyrieROS2LoadTestZelda.NUMBER_OF_PUBS;

   public static final String[] PUB_TOPIC_NAMES = getTopicNames();
   public static final String[] SUB_TOPIC_NAMES = ValkyrieROS2LoadTestZelda.PUB_TOPIC_NAMES;

   public static final long PUB_PERIOD_IN_MS = 1;
   public static final long SUB_PERIOD_IN_MS = 1000;

   public static final int MESSAGE_SIZE = 40;

   private final Random random = new Random();
   private final Ros2Node ros2node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ros2-load-test-link");
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1);

   private final List<IHMCROS2Publisher<KinematicsToolboxOutputStatus>> pubs = new ArrayList<>(NUMBER_OF_PUBS);
   private final List<AtomicReference<RobotConfigurationData>> subOutputs = new ArrayList<>(NUMBER_OF_SUBS);

   public ValkyrieROS2LoadTestLink()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(() -> shutdown()));

      for (int i = 0; i < NUMBER_OF_PUBS; i++)
      {
         pubs.add(ROS2Tools.createPublisher(ros2node, KinematicsToolboxOutputStatus.class, PUB_TOPIC_NAMES[i]));
      }

      for (int i = 0; i < NUMBER_OF_SUBS; i++)
      {
         AtomicReference<RobotConfigurationData> subOutput = new AtomicReference<>(null);
         subOutputs.add(subOutput);
         ROS2Tools.createCallbackSubscription(ros2node, RobotConfigurationData.class, SUB_TOPIC_NAMES[i], m -> subOutput.set(m.takeNextData()));
      }

      executorService.scheduleAtFixedRate(this::update, 0, PUB_PERIOD_IN_MS, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(this::printSubOutputs, 0, SUB_PERIOD_IN_MS, TimeUnit.MILLISECONDS);
   }

   private void update()
   {
      for (int i = 0; i < NUMBER_OF_PUBS; i++)
      {
         pubs.get(i).publish(nextKinematicsToolboxOutputStatus(random, MESSAGE_SIZE));
      }
   }

   private void printSubOutputs()
   {
      System.out.println();
      System.out.println();
      System.out.println("Most recent outputs at " + Conversions.nanosecondsToSeconds(System.nanoTime()));

      for (int i = 0; i < NUMBER_OF_SUBS; i++)
      {
         System.out.println(subOutputs.get(i).get());
      }
   }

   public static KinematicsToolboxOutputStatus nextKinematicsToolboxOutputStatus(Random random, int messageSize)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();

      message.getDesiredRootOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      message.getDesiredRootTranslation().set(EuclidCoreRandomTools.nextVector3D(random));
      message.getDesiredRootAngularVelocity().set(EuclidCoreRandomTools.nextVector3D(random));
      message.getDesiredRootLinearVelocity().set(EuclidCoreRandomTools.nextVector3D(random));

      for (int i = 0; i < messageSize; i++)
      {
         message.getDesiredJointAngles().add(random.nextFloat());
         message.getDesiredJointVelocities().add(random.nextFloat());
      }

      return message;
   }

   private static String[] getTopicNames()
   {
      String[] topicNames = new String[NUMBER_OF_PUBS];

      for (int i = 0; i < NUMBER_OF_PUBS; i++)
         topicNames[i] = "link_topic_" + i;
      return topicNames;
   }

   public void shutdown()
   {
      executorService.shutdown();
      ros2node.destroy();
   }

   public static void main(String[] args)
   {
      new ValkyrieROS2LoadTestLink();
   }
}
