package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFrameQuaternion;


public class ConstantPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final FramePoint3DBasics position;
   private final FrameQuaternionBasics orientation;
   private final ReferenceFrame referenceFrame;

   public ConstantPoseTrajectoryGenerator(FramePoint3DBasics position, FrameQuaternionBasics orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.referenceFrame = position.getReferenceFrame();
      this.position = position;
      this.orientation = orientation;
   }

   public ConstantPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this.referenceFrame = referenceFrame;

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      position = new YoMutableFramePoint3D(namePrefix + "ConstantPosition", "", registry, referenceFrame);
      orientation = new YoMutableFrameQuaternion(namePrefix + "ConstantOrientation", "", registry, referenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      position.changeFrame(referenceFrame);
      orientation.changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      position.setToZero(referenceFrame);
      orientation.setToZero(referenceFrame);
   }

   public void setConstantPose(FramePose3D constantPose)
   {
      position.checkReferenceFrameMatch(constantPose);
      position.set(constantPose.getX(), constantPose.getY(), constantPose.getZ());
      orientation.setYawPitchRoll(constantPose.getYaw(), constantPose.getPitch(), constantPose.getRoll());
   }

   public void setConstantPose(FramePoint3D constantPosition, FrameQuaternion constantOrientation)
   {
      this.position.set(position);
      this.orientation.set(orientation);
   }

   @Override
   public void initialize()
   {
      // Do nothing
   }

   @Override
   public void compute(double time)
   {
      // Do nothing
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void getVelocity(FixedFrameVector3DBasics velocityToPack)
   {
      velocityToPack.checkReferenceFrameMatch(position);
      velocityToPack.setToZero();
   }

   @Override
   public void getAcceleration(FixedFrameVector3DBasics accelerationToPack)
   {
      accelerationToPack.checkReferenceFrameMatch(accelerationToPack);
      accelerationToPack.setToZero();
   }

   @Override
   public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   @Override
   public void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.checkReferenceFrameMatch(orientation);
      angularVelocityToPack.setToZero();
   }

   @Override
   public void getAngularAcceleration(FixedFrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.checkReferenceFrameMatch(orientation);
      angularAccelerationToPack.setToZero();
   }

   @Override
   public void getPose(FixedFramePose3DBasics framePoseToPack)
   {
      getPosition(framePoseToPack.getPosition());
      getOrientation(framePoseToPack.getOrientation());
   }

   @Override
   public String toString()
   {
      String ret = "";
      ret += "Current position: " + position.toString();
      ret += "\nCurrent orientation: " + orientation.toString();
      return ret;
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}