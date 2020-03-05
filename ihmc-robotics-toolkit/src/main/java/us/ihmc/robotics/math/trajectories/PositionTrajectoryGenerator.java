package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator, PositionProvider
{
   void getVelocity(FixedFrameVector3DBasics velocityToPack);

   void getAcceleration(FixedFrameVector3DBasics accelerationToPack);

   default void getLinearData(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics velocityToPack, FixedFrameVector3DBasics accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   void showVisualization();

   void hideVisualization();
}
