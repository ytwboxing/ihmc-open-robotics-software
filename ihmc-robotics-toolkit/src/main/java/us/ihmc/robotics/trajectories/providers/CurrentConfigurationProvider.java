package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;

public class CurrentConfigurationProvider implements SE3ConfigurationProvider
{
   private final ReferenceFrame endEffectorFrame;

   public CurrentConfigurationProvider(ReferenceFrame endEffectorFrame)
   {
      this.endEffectorFrame = endEffectorFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return endEffectorFrame;
   }

   @Override
   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.checkReferenceFrameMatch(endEffectorFrame);
      positionToPack.setToZero();
   }

   @Override
   public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.checkReferenceFrameMatch(endEffectorFrame);
      orientationToPack.setToZero();
   }
}
