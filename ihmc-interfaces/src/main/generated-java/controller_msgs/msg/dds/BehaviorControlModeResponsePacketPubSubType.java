package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorControlModeResponsePacket" defined in "BehaviorControlModeResponsePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorControlModeResponsePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorControlModeResponsePacket_.idl instead.
*
*/
public class BehaviorControlModeResponsePacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BehaviorControlModeResponsePacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BehaviorControlModeResponsePacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BehaviorControlModeResponsePacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getBehaviorControlModeEnumRequest());

   }

   public static void read(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setBehaviorControlModeEnumRequest(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("behavior_control_mode_enum_request", data.getBehaviorControlModeEnumRequest());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BehaviorControlModeResponsePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setBehaviorControlModeEnumRequest(ser.read_type_9("behavior_control_mode_enum_request"));
   }

   public static void staticCopy(controller_msgs.msg.dds.BehaviorControlModeResponsePacket src, controller_msgs.msg.dds.BehaviorControlModeResponsePacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BehaviorControlModeResponsePacket createData()
   {
      return new controller_msgs.msg.dds.BehaviorControlModeResponsePacket();
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
   
   public void serialize(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BehaviorControlModeResponsePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BehaviorControlModeResponsePacket src, controller_msgs.msg.dds.BehaviorControlModeResponsePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorControlModeResponsePacketPubSubType newInstance()
   {
      return new BehaviorControlModeResponsePacketPubSubType();
   }
}
