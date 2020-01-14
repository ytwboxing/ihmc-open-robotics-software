package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;

public class DuplicatedPointCloud2
{
   private final long timestamp;
   private final sensor_msgs.msg.dds.PointCloud2 ros2PointCloud2;

   public DuplicatedPointCloud2(PointCloud2 rosPointCloud2, int maxSize)
   {
      timestamp = rosPointCloud2.getHeader().getStamp().totalNsecs();

      ros2PointCloud2 = new sensor_msgs.msg.dds.PointCloud2();
      ros2PointCloud2.setWidth(rosPointCloud2.getWidth());
      ros2PointCloud2.setHeight(rosPointCloud2.getHeight());
      ros2PointCloud2.setIsBigendian(true);
      ros2PointCloud2.setIsDense(true);
      int pointStep = rosPointCloud2.getPointStep();
      ros2PointCloud2.setPointStep(pointStep);
      ros2PointCloud2.setRowStep(rosPointCloud2.getRowStep());

      int numberOfPoints = rosPointCloud2.getWidth() * rosPointCloud2.getHeight();
      int arrayOffset = rosPointCloud2.getData().arrayOffset();
      ByteBuffer byteBuffer = ByteBuffer.wrap(rosPointCloud2.getData().array(), arrayOffset, numberOfPoints * pointStep);

      if (rosPointCloud2.getIsBigendian())
         byteBuffer.order(ByteOrder.BIG_ENDIAN);
      else
         byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

      for (int i = 0; i < 100; i++)
      {
         PointField ros1PointField = rosPointCloud2.getFields().get(i);
         sensor_msgs.msg.dds.PointField ros2PointField = new sensor_msgs.msg.dds.PointField();
         ros2PointField.setOffset(ros1PointField.getOffset());
         ros2PointField.setCount(ros1PointField.getCount());
         ros2PointField.setDatatype(ros1PointField.getDatatype());
         ros2PointCloud2.getFields().add(ros2PointField);

         byteBuffer.position(i * pointStep + arrayOffset);
         ros2PointCloud2.getData().add(byteBuffer.get());
         ros2PointCloud2.getData().add(byteBuffer.get());
         ros2PointCloud2.getData().add(byteBuffer.get());
      }
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public sensor_msgs.msg.dds.PointCloud2 getRos2PointCloud2()
   {
      return ros2PointCloud2;
   }

}
