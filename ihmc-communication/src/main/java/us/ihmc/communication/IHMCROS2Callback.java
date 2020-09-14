package us.ihmc.communication;

import geometry_msgs.msg.dds.PosePubSubType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.rosidl.geometry_msgs.msg.dds.Pose3DPubSubTypeImpl;

import java.util.function.Consumer;

/**
 * Callback listener to non-null reception of a message on a ROS 2 topic.
 *
 * @param <T> messageType
 */
public class IHMCROS2Callback<T>
{
   private final Consumer<T> messageCallback;
   private ROS2Subscription<T> subscription;
   private volatile boolean enabled = true;

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, ROS2Topic<T> topicName, Consumer<T> messageCallback)
   {
      this(ros2Node, topicName.getType(), topicName.getName(), messageCallback);
   }

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic<?> topicName, Consumer<T> messageCallback)
   {
      this(ros2Node, messageType, topicName.withTypeName(messageType).toString(), messageCallback);
   }

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName, Consumer<T> messageCallback)
   {
      this.messageCallback = messageCallback;
      ExceptionTools.handle(() ->
      {
         subscription = ros2Node.createSubscription(newMessageTopicDataTypeInstance(messageType), this::nullOmissionCallback, topicName);
      }, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   private static <T> TopicDataType<T> newMessageTopicDataTypeInstance(Class<T> messageType)
   {
      if (messageType.equals(Pose3D.class))
      {
         PosePubSubType.setImplementation(new Pose3DPubSubTypeImpl());
         return (TopicDataType<T>) new PosePubSubType();
      }
      else
      {
         return ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType);
      }
   }

   private void nullOmissionCallback(Subscriber<T> subscriber)
   {
      if (enabled)
      {
         ROS2Tools.takeNextDataAndCallbackIfNotNull(subscriber, messageCallback);
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public void destroy()
   {
      if (subscription != null)
      {
         subscription.remove();
      }
   }
}
