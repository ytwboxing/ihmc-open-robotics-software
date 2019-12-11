package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import controller_msgs.msg.dds.LidarScanMessage;
import geometry_msgs.Point;
import scan_to_cloud.PointCloud2WithSource;
import sensor_msgs.PointField;
import sensor_msgs.msg.dds.PointCloud2;
import std_msgs.msg.dds.Header;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class MultisensePointCloud2WithSourceROS1Bridge extends AbstractRosTopicSubscriber<PointCloud2WithSource>
{
   private static final MultisenseInformation multisense = MultisenseInformation.CART;
   
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "lidarScanPublisherNode");

   private final IHMCROS2Publisher<LidarScanMessage> lidarScanPublisher;
   private final IHMCROS2Publisher<PointCloud2> pointCloud2Publisher;

   public MultisensePointCloud2WithSourceROS1Bridge() throws URISyntaxException, IOException
   {
      super(PointCloud2WithSource._TYPE);
      URI masterURI = new URI(multisense.getAddress());
      RosMainNode rosMainNode = new RosMainNode(masterURI, "LidarScanPublisher", true);
      rosMainNode.attachSubscriber(MultisenseInformation.getLidarScanTopicName(), this);
      rosMainNode.execute();

      lidarScanPublisher = ROS2Tools.createPublisher(ros2Node, LidarScanMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      pointCloud2Publisher = ROS2Tools.createPublisher(ros2Node, PointCloud2.class, "/ihmc/multisense_lidar");
   }

   @Override
   public void onNewMessage(PointCloud2WithSource cloudHolder)
   {
      sensor_msgs.PointCloud2 ros1PointCloud2 = cloudHolder.getCloud();
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(ros1PointCloud2);
      Point3D[] points = pointCloudData.getPoints();

      Point translation = cloudHolder.getTranslation();
      Point3D lidarPosition = new Point3D(translation.getX(), translation.getY(), translation.getZ());
      geometry_msgs.Quaternion orientation = cloudHolder.getOrientation();
      Quaternion lidarQuaternion = new Quaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getW());

      PointCloud2 pointCloud2 = new PointCloud2();
      pointCloud2.getHeader().setFrameId(ros1PointCloud2.getHeader().getFrameId());
      pointCloud2.setHeight(ros1PointCloud2.getHeight());
      pointCloud2.setWidth(ros1PointCloud2.getWidth());
      for (PointField field : ros1PointCloud2.getFields())
      {
         sensor_msgs.msg.dds.PointField pointField = new sensor_msgs.msg.dds.PointField();
         pointField.setName(field.getName());
         pointField.setOffset(field.getOffset());
         pointField.setDatatype(field.getDatatype());
         pointField.setCount(field.getCount());
         pointCloud2.getFields().add(pointField);
      }
      pointCloud2.setIsBigendian(ros1PointCloud2.getIsBigendian());
      pointCloud2.setPointStep(ros1PointCloud2.getPointStep());
      pointCloud2.setRowStep(ros1PointCloud2.getRowStep());
      for (byte b : ros1PointCloud2.getData().array())
      {
         pointCloud2.getData().add(b);
      }
      pointCloud2.setIsDense(ros1PointCloud2.getIsDense());
      pointCloud2Publisher.publish(pointCloud2);

      LidarScanMessage lidarScanMessage = new LidarScanMessage();
      lidarScanMessage.getLidarPosition().set(lidarPosition);
      lidarScanMessage.getLidarOrientation().set(lidarQuaternion);
      MessageTools.packScan(lidarScanMessage, points);

      lidarScanPublisher.publish(lidarScanMessage);
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisensePointCloud2WithSourceROS1Bridge();
   }
}
