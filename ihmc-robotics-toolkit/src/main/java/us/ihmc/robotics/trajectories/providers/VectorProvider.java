package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;

public interface VectorProvider
{
   void get(FixedFrameVector3DBasics frameVectorToPack);
}
