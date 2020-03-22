package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface OrientationProvider extends ReferenceFrameHolder
{
   default void getOrientation(FrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setReferenceFrame(this.getReferenceFrame());
      getOrientation((FixedFrameQuaternionBasics) orientationToPack);
   }

   void getOrientation(FixedFrameQuaternionBasics orientationToPack);
}