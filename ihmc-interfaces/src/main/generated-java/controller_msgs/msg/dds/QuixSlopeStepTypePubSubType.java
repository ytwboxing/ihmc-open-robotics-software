package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuixSlopeStepType" defined in "QuixSlopeStepType_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuixSlopeStepType_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuixSlopeStepType_.idl instead.
*
*/
public class QuixSlopeStepTypePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuixSlopeStepType>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuixSlopeStepType_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuixSlopeStepType data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuixSlopeStepType data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuixSlopeStepType data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuixSlopeStepType data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuixSlopeStepType data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_9(data.getDesiredSlopeStepType());

   }

   public static void read(controller_msgs.msg.dds.QuixSlopeStepType data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setDesiredSlopeStepType(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuixSlopeStepType data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_9("desired_slope_step_type", data.getDesiredSlopeStepType());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuixSlopeStepType data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setDesiredSlopeStepType(ser.read_type_9("desired_slope_step_type"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuixSlopeStepType src, controller_msgs.msg.dds.QuixSlopeStepType dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuixSlopeStepType createData()
   {
      return new controller_msgs.msg.dds.QuixSlopeStepType();
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
   
   public void serialize(controller_msgs.msg.dds.QuixSlopeStepType data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuixSlopeStepType data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuixSlopeStepType src, controller_msgs.msg.dds.QuixSlopeStepType dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuixSlopeStepTypePubSubType newInstance()
   {
      return new QuixSlopeStepTypePubSubType();
   }
}
