package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;

public class WrapperForPositionAndOrientationTrajectoryGenerators implements PoseTrajectoryGenerator
{
   private final PositionTrajectoryGenerator positionTrajectoryGenerator;
   private final OrientationTrajectoryGenerator orientationTrajectoryGenerator;

   public WrapperForPositionAndOrientationTrajectoryGenerators(PositionTrajectoryGenerator positionTrajectoryGenerator,
                                                               OrientationTrajectoryGenerator orientationTrajectoryGenerator)
   {
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.orientationTrajectoryGenerator = orientationTrajectoryGenerator;
   }

   public void initialize()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   public void compute(double time)
   {
      positionTrajectoryGenerator.compute(time);
      orientationTrajectoryGenerator.compute(time);
   }

   public boolean isDone()
   {
      return positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return positionTrajectoryGenerator.getReferenceFrame();
   }

   @Override
   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionTrajectoryGenerator.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FixedFrameVector3DBasics velocityToPack)
   {
      positionTrajectoryGenerator.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FixedFrameVector3DBasics accelerationToPack)
   {
      positionTrajectoryGenerator.getAcceleration(accelerationToPack);
   }

   @Override
   public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      orientationTrajectoryGenerator.getAngularVelocity(angularVelocityToPack);
   }

   @Override
   public void getAngularAcceleration(FixedFrameVector3DBasics angularAccelerationToPack)
   {
      orientationTrajectoryGenerator.getAngularAcceleration(angularAccelerationToPack);
   }

   public void showVisualization()
   {
      positionTrajectoryGenerator.showVisualization();
   }

   public void hideVisualization()
   {
      positionTrajectoryGenerator.hideVisualization();
   }
}
