package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       * This message is used to notify the controller of the user's desired flat step type
       */
public class QuixFlatStepType extends Packet<QuixFlatStepType> implements Settable<QuixFlatStepType>, EpsilonComparable<QuixFlatStepType>
{

   public static final byte SQUARE = (byte) 0;

   public static final byte SHORT_FORWARD = (byte) 1;

   public static final byte MEDIUM_FORWARD = (byte) 2;

   public static final byte LONG_FORWARD = (byte) 3;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte desired_flat_step_type_ = (byte) 255;

   public QuixFlatStepType()
   {



   }

   public QuixFlatStepType(QuixFlatStepType other)
   {
      this();
      set(other);
   }

   public void set(QuixFlatStepType other)
   {

      sequence_id_ = other.sequence_id_;


      desired_flat_step_type_ = other.desired_flat_step_type_;

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setDesiredFlatStepType(byte desired_flat_step_type)
   {
      desired_flat_step_type_ = desired_flat_step_type;
   }
   public byte getDesiredFlatStepType()
   {
      return desired_flat_step_type_;
   }


   public static Supplier<QuixFlatStepTypePubSubType> getPubSubType()
   {
      return QuixFlatStepTypePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixFlatStepTypePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixFlatStepType other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_flat_step_type_, other.desired_flat_step_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixFlatStepType)) return false;

      QuixFlatStepType otherMyClass = (QuixFlatStepType) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.desired_flat_step_type_ != otherMyClass.desired_flat_step_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixFlatStepType {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("desired_flat_step_type=");
      builder.append(this.desired_flat_step_type_);
      builder.append("}");
      return builder.toString();
   }
}
