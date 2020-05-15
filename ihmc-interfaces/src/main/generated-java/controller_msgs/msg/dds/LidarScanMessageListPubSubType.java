package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LidarScanMessageList" defined in "LidarScanMessageList_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LidarScanMessageList_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LidarScanMessageList_.idl instead.
*
*/
public class LidarScanMessageListPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.LidarScanMessageList>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::LidarScanMessageList_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.LidarScanMessageList data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.LidarScanMessageList data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 500; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.LidarScanMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LidarScanMessageList data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LidarScanMessageList data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getMessages().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.LidarScanMessagePubSubType.getCdrSerializedSize(data.getMessages().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.LidarScanMessageList data, us.ihmc.idl.CDR cdr)
   {

      if(data.getMessages().size() <= 500)
      cdr.write_type_e(data.getMessages());else
          throw new RuntimeException("messages field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.LidarScanMessageList data, us.ihmc.idl.CDR cdr)
   {

      cdr.read_type_e(data.getMessages());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.LidarScanMessageList data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_e("messages", data.getMessages());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.LidarScanMessageList data)
   {

      ser.read_type_e("messages", data.getMessages());
   }

   public static void staticCopy(controller_msgs.msg.dds.LidarScanMessageList src, controller_msgs.msg.dds.LidarScanMessageList dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.LidarScanMessageList createData()
   {
      return new controller_msgs.msg.dds.LidarScanMessageList();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.LidarScanMessageList data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.LidarScanMessageList data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.LidarScanMessageList src, controller_msgs.msg.dds.LidarScanMessageList dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LidarScanMessageListPubSubType newInstance()
   {
      return new LidarScanMessageListPubSubType();
   }
}
