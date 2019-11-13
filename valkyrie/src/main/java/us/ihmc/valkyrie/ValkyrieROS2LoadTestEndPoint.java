package us.ihmc.valkyrie;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

@SuppressWarnings({"rawtypes", "unchecked"})
public class ValkyrieROS2LoadTestEndPoint
{
   public final String[] pubTopicNames, subTopicNames;

   private final Random random = new Random();
   private final Ros2Node pubROS2node;
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1);

   private final List<IHMCROS2Publisher> pubs;
   private final List<AtomicReference> subOutputs;

   private final EndPointParameters endPointParameters;

   public ValkyrieROS2LoadTestEndPoint(EndPointParameters endPointParameters, EndPointParameters otherEndPointParameters) throws IOException
   {
      this.endPointParameters = endPointParameters;
      Runtime.getRuntime().addShutdownHook(new Thread(() -> shutdown()));

      pubROS2node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ros2_load_test_pub_node_" + endPointParameters.getName());
      pubTopicNames = topicNames(endPointParameters);
      subTopicNames = topicNames(otherEndPointParameters);
      int numberOfPubs = endPointParameters.getNumberOfPubs();
      int numberOfSubs = endPointParameters.getNumberOfSubsPerTopic() * otherEndPointParameters.getNumberOfPubs();
      pubs = new ArrayList<>(numberOfPubs);
      subOutputs = new ArrayList<>(numberOfSubs);

      for (int i = 0; i < numberOfPubs; i++)
      {
         IHMCROS2Publisher publisher = ROS2Tools.createPublisher(pubROS2node, endPointParameters.getPubType(), pubTopicNames[i]);
         pubs.add(publisher);
      }

      for (int i = 0; i < otherEndPointParameters.getNumberOfPubs(); i++)
      {
         String topicName = subTopicNames[i];
         Class subType = otherEndPointParameters.getPubType();

         for (int j = 0; j < endPointParameters.getNumberOfSubsPerTopic(); j++)
         {
            AtomicReference subOutput = new AtomicReference<>(null);
            subOutputs.add(subOutput);
            ROS2Tools.createCallbackSubscription(pubROS2node, subType, topicName, m -> subOutput.set(m.takeNextData()));
         }
      }

      long pubUpdatePeriodInMs = endPointParameters.getPubUpdatePeriodInMs();
      long subUpdatePeriodInMs = endPointParameters.getSubUpdatePeriodInMs();

      executorService.scheduleAtFixedRate(this::update, 0, pubUpdatePeriodInMs, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(this::printSubOutputs, 0, subUpdatePeriodInMs, TimeUnit.MILLISECONDS);
   }

   private void update()
   {
      for (int i = 0; i < pubs.size(); i++)
      {
         pubs.get(i).publish(endPointParameters.nextMessage(random));
      }
   }

   private void printSubOutputs()
   {
      System.out.println();
      System.out.println();
      System.out.println("Most recent outputs at " + Conversions.nanosecondsToSeconds(System.nanoTime()));

      for (int i = 0; i < subOutputs.size(); i++)
      {
         Object message = subOutputs.get(i).getAndSet(null);
         System.out.println(message == null ? "null" : message.getClass().getSimpleName());
      }
   }

   private void shutdown()
   {
      executorService.shutdown();
      pubROS2node.destroy();
      System.exit(0);
   }

   private String[] topicNames(EndPointParameters parameters)
   {
      String[] topicNames = new String[parameters.getNumberOfPubs()];

      for (int i = 0; i < parameters.getNumberOfPubs(); i++)
         topicNames[i] = parameters.getName() + "_topic_" + i;

      return topicNames;
   }

   public static class EndPointParameters
   {
      private final String name;
      private final int numberOfPubs;
      private final int numberOfSubsPerTopic;
      private final long pubUpdatePeriodInMs;
      private final long subUpdatePeriodInMs;
      private final Function<Random, Object> randomGenerator;

      public EndPointParameters(String name, int numberOfPubs, int numberOfSubsPerTopic, long pubUpdatePeriodInMs, long subUpdatePeriodInMs,
                                Function<Random, Object> randomGenerator)
      {
         this.name = name;
         this.numberOfPubs = numberOfPubs;
         this.numberOfSubsPerTopic = numberOfSubsPerTopic;
         this.pubUpdatePeriodInMs = pubUpdatePeriodInMs;
         this.subUpdatePeriodInMs = subUpdatePeriodInMs;
         this.randomGenerator = randomGenerator;
      }

      public String getName()
      {
         return name;
      }

      public int getNumberOfPubs()
      {
         return numberOfPubs;
      }

      public int getNumberOfSubsPerTopic()
      {
         return numberOfSubsPerTopic;
      }

      public long getPubUpdatePeriodInMs()
      {
         return pubUpdatePeriodInMs;
      }

      public long getSubUpdatePeriodInMs()
      {
         return subUpdatePeriodInMs;
      }

      public Object nextMessage(Random random)
      {
         return randomGenerator.apply(random);
      }

      public Class<?> getPubType()
      {
         return nextMessage(new Random()).getClass();
      }
   }

   public static EndPointParameters linkEndPoint()
   {
      return new EndPointParameters("link", 1, 1, 1, 5000, ValkyrieROS2LoadTestEndPoint::nextRobotConfigurationData);
   }

   public static EndPointParameters zeldaEndPoint()
   {
      return new EndPointParameters("zelda", 1, 1, 1, 5000, ValkyrieROS2LoadTestEndPoint::nextKinematicsToolboxOutputStatus);
   }

   public static KinematicsToolboxOutputStatus nextKinematicsToolboxOutputStatus(Random random)
   {
      KinematicsToolboxOutputStatus message = new KinematicsToolboxOutputStatus();

      message.getDesiredRootOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      message.getDesiredRootTranslation().set(EuclidCoreRandomTools.nextVector3D(random));
      message.getDesiredRootAngularVelocity().set(EuclidCoreRandomTools.nextVector3D(random));
      message.getDesiredRootLinearVelocity().set(EuclidCoreRandomTools.nextVector3D(random));

      for (int i = 40; i < 0; i++)
      {
         message.getDesiredJointAngles().add(random.nextFloat());
         message.getDesiredJointVelocities().add(random.nextFloat());
      }

      return message;
   }

   public static RobotConfigurationData nextRobotConfigurationData(Random random)
   {
      RobotConfigurationData message = new RobotConfigurationData();

      message.getRootTranslation().set(EuclidCoreRandomTools.nextVector3D(random));
      message.getRootOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));

      for (int i = 40; i < 0; i++)
      {
         message.getJointAngles().add(random.nextFloat());
         message.getJointVelocities().add(random.nextFloat());
         message.getJointTorques().add(random.nextFloat());
      }
      return message;
   }

   public static StereoVisionPointCloudMessage nextStereoVisionPointCloudMessage(Random random)
   {
      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();

      for (int i = 0; i < 200000; i++)
      {
         message.getPointCloud().add(random.nextFloat());
         message.getPointCloud().add(random.nextFloat());
         message.getPointCloud().add(random.nextFloat());
         message.getColors().add(random.nextInt());
      }

      return message;
   }
}
