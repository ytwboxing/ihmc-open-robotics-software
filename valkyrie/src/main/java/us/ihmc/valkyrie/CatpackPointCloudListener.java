package us.ihmc.valkyrie;

import controller_msgs.msg.dds.*;
import geometry_msgs.msg.dds.PoseStamped;
import geometry_msgs.msg.dds.PoseStampedPubSubType;
import gnu.trove.list.array.TByteArrayList;
import org.apache.commons.lang3.mutable.MutableInt;
import org.bytedeco.javacv.FrameFilter.Exception;
import sensor_msgs.msg.dds.PointCloud2;
import sensor_msgs.msg.dds.PointCloud2PubSubType;
import sensor_msgs.msg.dds.PointField;
import tf2_msgs.msg.dds.TFMessage;
import tf2_msgs.msg.dds.TFMessagePubSubType;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2QosProfile;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class CatpackPointCloudListener
{
   public CatpackPointCloudListener() throws IOException
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "catpack_point_cloud");

      AtomicReference<PointCloud2> pointCloudReference = new AtomicReference<>();
      AtomicReference<PoseStamped> poseStampedReference = new AtomicReference<>();
      MutableInt counter = new MutableInt();

      String pointCloudTopicName = "/slam/odom/cloud";
      PointCloud2PubSubType pointCloudDataType = new PointCloud2PubSubType();
      NewMessageListener<PointCloud2> pointCloudCallback = s ->
      {
         PointCloud2 pointCloud2 = s.readNextData();
         pointCloudReference.set(pointCloud2);
      };

      String poseTopicName = "/slam/odom/pose";
      PoseStampedPubSubType poseDataType = new PoseStampedPubSubType();
      NewMessageListener<PoseStamped> poseCallback = s ->
      {
         PoseStamped poseMessage = s.readNextData();
         poseStampedReference.set(poseMessage);
      };

      Ros2QosProfile rosQoSProfile = Ros2QosProfile.DEFAULT();
      ros2Node.createSubscription(pointCloudDataType, pointCloudCallback, pointCloudTopicName, rosQoSProfile);
      ros2Node.createSubscription(poseDataType, poseCallback, poseTopicName, rosQoSProfile);

      ThreadFactory threadFactory = ThreadTools.createNamedThreadFactory(getClass().getSimpleName());
      ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1, threadFactory);
      IHMCROS2Publisher<LidarScanMessage> ihmcPointCloudPublisher = ROS2Tools.createPublisher(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan");

      LidarScanMessageList messageList = new LidarScanMessageList();
      int numScans = 300;

      Runnable ihmcPointCloudRunnable = () ->
      {
         if (poseStampedReference.get() == null || pointCloudReference.get() == null)
         {
            return;
         }

         PointCloud2 pointCloud = pointCloudReference.getAndSet(null);
         PoseStamped pose = poseStampedReference.get();

         int width = (int) pointCloud.getWidth();
         int height = (int) pointCloud.getHeight();
         int numberOfPoints = width * height;
         int numPointsToConsider = Math.min((PointCloud2.NUMBER_OF_POINTS / 26) - 1, numberOfPoints);

         System.out.println("received -- " + counter.getValue());

         float[] points = new float[3 * numPointsToConsider];
         byte[] bytes = new byte[4];

         for (int i = 0; i < numPointsToConsider; i++)
         {
            int startIndex = 26 * i;
            packBytes(bytes, startIndex + 0, pointCloud.getData());
            float x = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            packBytes(bytes, startIndex + 4, pointCloud.getData());
            float y = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            packBytes(bytes, startIndex + 8, pointCloud.getData());
            float z = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            points[3 * i + 0] = x;
            points[3 * i + 1] = y;
            points[3 * i + 2] = z;
         }

         LidarScanMessage message = messageList.getMessages().add();
         message.getLidarPosition().set(pose.getPose().getPosition());
         message.getLidarOrientation().set(pose.getPose().getOrientation());
         message.setSequenceId(counter.getAndIncrement());
         message.getScan().add(points);

//         ihmcPointCloudPublisher.publish(message);

         if (counter.getValue() > 1500)
         {
            try
            {
               String file = System.getProperty("user.home") + File.separator + "LidarScanMessageList.json";
               JSONSerializer<LidarScanMessageList> serializer = new JSONSerializer<>(new LidarScanMessageListPubSubType());
               byte[] serializedFootstepParameters = serializer.serializeToBytes(messageList);

               FileTools.ensureFileExists(new File(file).toPath());
               FileOutputStream outputStream = new FileOutputStream(file);
               PrintStream printStream = new PrintStream(outputStream);

               printStream.write(serializedFootstepParameters);
               printStream.flush();
               outputStream.close();
               printStream.close();
               System.exit(0);
            }
            catch (IOException e)
            {
               e.printStackTrace();
               System.exit(1);
            }
         }
      };

      executorService.scheduleAtFixedRate(ihmcPointCloudRunnable, 0, 5, TimeUnit.MILLISECONDS);
   }

   static void packBytes(byte[] bytes, int startIndex, TByteArrayList data)
   {
      for (int i = 0; i < 4; i++)
      {
         bytes[i] = data.get(startIndex + i);
      }
   }

   public static void main(String[] args) throws IOException
   {
      new CatpackPointCloudListener();
   }
}
