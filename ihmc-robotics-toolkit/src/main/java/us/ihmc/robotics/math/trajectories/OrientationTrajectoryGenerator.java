package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public interface OrientationTrajectoryGenerator extends TrajectoryGenerator, OrientationProvider
{
   default void getAngularVelocity(FrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setReferenceFrame(getReferenceFrame());
      getAngularVelocity((FixedFrameVector3DBasics) angularVelocityToPack);
   }

   void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack);

   default void getAngularAcceleration(FrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.setReferenceFrame(getReferenceFrame());
      getAngularAcceleration((FixedFrameVector3DBasics) angularAccelerationToPack);
   }

   void getAngularAcceleration(FixedFrameVector3DBasics angularAccelerationToPack);

   default void getAngularData(FrameQuaternionBasics orientationToPack, FrameVector3DBasics angularVelocityToPack, FrameVector3DBasics angularAccelerationToPack)
   {
      getOrientation((FrameQuaternionBasics) orientationToPack);
      getAngularVelocity((FrameVector3DBasics) angularVelocityToPack);
      getAngularAcceleration((FrameVector3DBasics) angularAccelerationToPack);
   }

   default void getAngularData(FixedFrameQuaternionBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack,
                                      FixedFrameVector3DBasics angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}