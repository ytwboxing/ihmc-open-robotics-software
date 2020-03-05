package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;

public interface OrientationProvider
{
   void getOrientation(FixedFrameQuaternionBasics orientationToPack);
}