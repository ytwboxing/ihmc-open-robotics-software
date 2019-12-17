package us.ihmc.atlas;

import scan_to_cloud.PointCloud2WithSource;
import sensor_msgs.PointField;
import sensor_msgs.msg.dds.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.net.URI;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class AtlasMultisenseROS2Bridge
{
   // subscribe to mutlisense ros 1
   // publish pointcloud2 ROS 2

   public AtlasMultisenseROS2Bridge()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(FAST_RTPS, "atlas_multisense_ros1_bridge");

      IHMCROS2Publisher<PointCloud2> pointCloud2Publisher = ROS2Tools.createPublisher(ros2Node, PointCloud2.class, "/ihmc/multisense_lidar");

      RosMainNode rosMainNode = new RosMainNode(URI.create("http://172.16.66.100:11311"), "atlas/sensorSuiteManager", true);

      String multisense_namespace = "/multisense";
      String lidarPoseLink = "hokuyo_link";
      String lidarJointName = "hokuyo_joint";
      String lidarEndFrameInSdf = "/head_hokuyo_frame";
      String lidarBaseFrame = multisense_namespace + "/head_root";
      String lidarEndFrame = multisense_namespace + lidarEndFrameInSdf;
      String lidarSensorName = "head_hokuyo_sensor";
      String lidarJointTopic = multisense_namespace + "/joint_states";
      String multisense_laser_topic_string = multisense_namespace + "/lidar_scan";
      String multisense_laser_scan_topic_string = "/singleScanAsCloudWithSource";
      String multisense_laser_topic__as_string = multisense_namespace + "/lidar_points2";
      String multisense_filtered_laser_as_point_cloud_topic_string = multisense_namespace + "/filtered_cloud";
      String multisense_ground_point_cloud_topic_string = multisense_namespace + "/highly_filtered_cloud";
      String bodyIMUSensor = "pelvis_imu_sensor_at_pelvis_frame";
      String chestIMUSensor = "utorso_imu_sensor_chest";

      rosMainNode.attachSubscriber(multisense_laser_scan_topic_string, new AbstractRosTopicSubscriber<PointCloud2WithSource>(PointCloud2WithSource._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2WithSource pointCloud)
         {
            sensor_msgs.PointCloud2 ros1PointCloud2 = pointCloud.getCloud();

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

            LogTools.info("Publishing {} points", pointCloud2.width_);
            pointCloud2Publisher.publish(pointCloud2);
         }
      });

      rosMainNode.execute();

      ThreadTools.join();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new AtlasMultisenseROS2Bridge();
   }
}
