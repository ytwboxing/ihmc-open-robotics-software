package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       * This message is used to notify the controller of the user's desired slope step type
       */
public class QuixSlopeStepType extends Packet<QuixSlopeStepType> implements Settable<QuixSlopeStepType>, EpsilonComparable<QuixSlopeStepType>
{

   public static final byte SLOPE_FLAT = (byte) 0;

   public static final byte SLOPE_UP = (byte) 1;

   public static final byte SLOPE_DOWN = (byte) 2;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte desired_slope_step_type_ = (byte) 255;

   public QuixSlopeStepType()
   {



   }

   public QuixSlopeStepType(QuixSlopeStepType other)
   {
      this();
      set(other);
   }

   public void set(QuixSlopeStepType other)
   {

      sequence_id_ = other.sequence_id_;


      desired_slope_step_type_ = other.desired_slope_step_type_;

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


   public void setDesiredSlopeStepType(byte desired_slope_step_type)
   {
      desired_slope_step_type_ = desired_slope_step_type;
   }
   public byte getDesiredSlopeStepType()
   {
      return desired_slope_step_type_;
   }


   public static Supplier<QuixSlopeStepTypePubSubType> getPubSubType()
   {
      return QuixSlopeStepTypePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixSlopeStepTypePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixSlopeStepType other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_slope_step_type_, other.desired_slope_step_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixSlopeStepType)) return false;

      QuixSlopeStepType otherMyClass = (QuixSlopeStepType) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.desired_slope_step_type_ != otherMyClass.desired_slope_step_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixSlopeStepType {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("desired_slope_step_type=");
      builder.append(this.desired_slope_step_type_);
      builder.append("}");
      return builder.toString();
   }
}
