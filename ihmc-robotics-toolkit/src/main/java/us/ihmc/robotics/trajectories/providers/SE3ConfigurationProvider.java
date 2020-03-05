package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;

public interface SE3ConfigurationProvider extends PositionProvider, OrientationProvider
{
   default void getPose(FixedFramePose3DBasics lastFootstepPose)
   {
      getPosition(lastFootstepPose.getPosition());
      getOrientation(lastFootstepPose.getOrientation());
   }
}
