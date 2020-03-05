package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;

public interface FramePoseProvider
{
   void getPose(FixedFramePose3DBasics framePoseToPack);
}