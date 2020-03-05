package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.robotics.trajectories.providers.FramePoseProvider;


public interface PoseTrajectoryGenerator extends PositionTrajectoryGenerator, OrientationTrajectoryGenerator, FramePoseProvider
{
   default void getPose(FixedFramePose3DBasics framePoseToPack)
   {
      getPosition(framePoseToPack.getPosition());
      getOrientation(framePoseToPack.getOrientation());
   }
}
