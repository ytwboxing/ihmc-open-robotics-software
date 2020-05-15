package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LidarScanMessageList extends Packet<LidarScanMessageList> implements Settable<LidarScanMessageList>, EpsilonComparable<LidarScanMessageList>
{

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.LidarScanMessage>  messages_;

   public LidarScanMessageList()
   {

      messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.LidarScanMessage> (500, new controller_msgs.msg.dds.LidarScanMessagePubSubType());

   }

   public LidarScanMessageList(LidarScanMessageList other)
   {
      this();
      set(other);
   }

   public void set(LidarScanMessageList other)
   {

      messages_.set(other.messages_);
   }



   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.LidarScanMessage>  getMessages()
   {
      return messages_;
   }


   public static Supplier<LidarScanMessageListPubSubType> getPubSubType()
   {
      return LidarScanMessageListPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LidarScanMessageListPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LidarScanMessageList other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (this.messages_.size() != other.messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.messages_.size(); i++)
         {  if (!this.messages_.get(i).epsilonEquals(other.messages_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LidarScanMessageList)) return false;

      LidarScanMessageList otherMyClass = (LidarScanMessageList) other;


      if (!this.messages_.equals(otherMyClass.messages_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LidarScanMessageList {");

      builder.append("messages=");
      builder.append(this.messages_);
      builder.append("}");
      return builder.toString();
   }
}
