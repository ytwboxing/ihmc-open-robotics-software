package us.ihmc.quadrupedCommunication;

import controller_msgs.msg.dds.QuadrupedTeleopMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;

public class QuadrupedTeleopCommand implements Command<QuadrupedTeleopCommand, QuadrupedTeleopMessage>
{
   private long sequenceId;
   private boolean areStepsAdjustable;
   private final Vector3D desiredVelocity = new Vector3D();
   private final QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();

   @Override
   public void clear()
   {
      sequenceId = 0;
      areStepsAdjustable = true;
      desiredVelocity.setToZero();
   }

   @Override
   public void setFromMessage(QuadrupedTeleopMessage message)
   {
      sequenceId = message.getSequenceId();
      areStepsAdjustable = message.getAreStepsAdjustable();
      desiredVelocity.set(message.getDesiredVelocity());
      xGaitSettings.set(message.getXGaitSettings());
   }

   @Override
   public Class<QuadrupedTeleopMessage> getMessageClass()
   {
      return QuadrupedTeleopMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !desiredVelocity.containsNaN();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(QuadrupedTeleopCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.areStepsAdjustable = other.areStepsAdjustable;
      this.desiredVelocity.set(other.desiredVelocity);
      this.xGaitSettings.set(other.xGaitSettings);
   }

   public boolean areStepsAdjustable()
   {
      return areStepsAdjustable;
   }

   public Tuple3DReadOnly getDesiredVelocity()
   {
      return desiredVelocity;
   }

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      return xGaitSettings;
   }
}