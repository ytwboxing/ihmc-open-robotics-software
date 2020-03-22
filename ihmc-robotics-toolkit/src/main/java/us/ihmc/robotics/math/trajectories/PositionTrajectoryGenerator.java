package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator, PositionProvider
{
   default void getVelocity(FrameVector3DBasics velocityToPack)
   {
      velocityToPack.setReferenceFrame(this.getReferenceFrame());
      getVelocity((FixedFrameVector3DBasics) velocityToPack);
   }

   void getVelocity(FixedFrameVector3DBasics velocityToPack);

   default void getAcceleration(FrameVector3DBasics accelerationToPack)
   {
      accelerationToPack.setReferenceFrame(this.getReferenceFrame());
      getAcceleration((FixedFrameVector3DBasics) accelerationToPack);
   }

   void getAcceleration(FixedFrameVector3DBasics accelerationToPack);

   default void getLinearData(FramePoint3DBasics positionToPack, FrameVector3DBasics velocityToPack, FrameVector3DBasics accelerationToPack)
   {
      getPosition((FramePoint3DBasics) positionToPack);
      getVelocity((FrameVector3DBasics) velocityToPack);
      getAcceleration((FrameVector3DBasics) accelerationToPack);
   }

   default void getLinearData(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics velocityToPack, FixedFrameVector3DBasics accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   void showVisualization();

   void hideVisualization();
}
