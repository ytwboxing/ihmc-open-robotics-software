package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface FramePoseProvider extends ReferenceFrameHolder
{
   default void getPose(FramePose3DBasics framePoseToPack)
   {
      framePoseToPack.setReferenceFrame(this.getReferenceFrame());
      getPose((FixedFramePose3DBasics) framePoseToPack);
   }

   void getPose(FixedFramePose3DBasics framePoseToPack);
}