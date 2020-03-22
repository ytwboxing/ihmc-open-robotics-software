package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BlendedPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final BlendedPositionTrajectoryGenerator blendedPositionTrajectory;
   private final BlendedOrientationTrajectoryGenerator blendedOrientationTrajectory;

   private final PoseTrajectoryGenerator trajectory;

   public BlendedPoseTrajectoryGenerator(String prefix, PoseTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame, YoVariableRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.blendedPositionTrajectory = new BlendedPositionTrajectoryGenerator(prefix + "Position", trajectory, trajectoryFrame, parentRegistry);
      this.blendedOrientationTrajectory = new BlendedOrientationTrajectoryGenerator(prefix + "Orientation", trajectory, trajectoryFrame, parentRegistry);
   }

   public void clear()
   {
      blendedPositionTrajectory.clear();
      blendedOrientationTrajectory.clear();
   }

   public void clearInitialConstraint()
   {
      blendedPositionTrajectory.clearInitialConstraint();
      blendedOrientationTrajectory.clearInitialConstraint();
   }

   public void clearFinalConstraint()
   {
      blendedPositionTrajectory.clearFinalConstraint();
      blendedOrientationTrajectory.clearFinalConstraint();
   }

   public void blendInitialConstraint(FramePose3DReadOnly initialPose, double initialTime, double blendDuration)
   {
      blendedPositionTrajectory.blendInitialConstraint(initialPose.getPosition(), initialTime, blendDuration);
      blendedOrientationTrajectory.blendInitialConstraint(initialPose.getOrientation(), initialTime, blendDuration);
   }

   public void blendInitialConstraint(FramePose3DReadOnly initialPose, TwistReadOnly initialTwist, double initialTime, double blendDuration)
   {
      blendedPositionTrajectory.blendInitialConstraint(initialPose.getPosition(), initialTwist.getLinearPart(), initialTime, blendDuration);
      blendedOrientationTrajectory.blendInitialConstraint(initialPose.getOrientation(), initialTwist.getAngularPart(), initialTime, blendDuration);
   }

   public void blendFinalConstraint(FramePose3DReadOnly finalPose, double finalTime, double blendDuration)
   {
      blendedPositionTrajectory.blendFinalConstraint(finalPose.getPosition(), finalTime, blendDuration);
      blendedOrientationTrajectory.blendFinalConstraint(finalPose.getOrientation(), finalTime, blendDuration);
   }

   public void blendFinalConstraint(FramePose3DReadOnly finalPose, TwistReadOnly finalTwist, double finalTime, double blendDuration)
   {
      blendedPositionTrajectory.blendFinalConstraint(finalPose.getPosition(), finalTwist.getLinearPart(), finalTime, blendDuration);
      blendedOrientationTrajectory.blendFinalConstraint(finalPose.getOrientation(), finalTwist.getAngularPart(), finalTime, blendDuration);
   }

   public void initializeTrajectory()
   {
      trajectory.initialize();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return trajectory.getReferenceFrame();
   }

   @Override
   public void getPose(FixedFramePose3DBasics framePoseToPack)
   {
      blendedPositionTrajectory.getPosition(framePoseToPack.getPosition());
      blendedOrientationTrajectory.getOrientation(framePoseToPack.getOrientation());
   }

   @Override
   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      blendedPositionTrajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FixedFrameVector3DBasics velocityToPack)
   {
      blendedPositionTrajectory.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FixedFrameVector3DBasics accelerationToPack)
   {
      blendedPositionTrajectory.getAcceleration(accelerationToPack);
   }

   @Override
   public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      blendedOrientationTrajectory.getOrientation(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      blendedOrientationTrajectory.getAngularVelocity(angularVelocityToPack);
   }

   @Override
   public void getAngularAcceleration(FixedFrameVector3DBasics angularAccelerationToPack)
   {
      blendedOrientationTrajectory.getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      blendedPositionTrajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      blendedPositionTrajectory.hideVisualization();
   }

   @Override
   public void initialize()
   {
      blendedPositionTrajectory.initialize();
      blendedOrientationTrajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      blendedPositionTrajectory.compute(time);
      blendedOrientationTrajectory.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return blendedPositionTrajectory.isDone() && blendedOrientationTrajectory.isDone();
   }
}
