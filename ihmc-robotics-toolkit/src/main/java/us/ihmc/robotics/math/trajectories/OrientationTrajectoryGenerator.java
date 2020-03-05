package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public interface OrientationTrajectoryGenerator extends TrajectoryGenerator, OrientationProvider
{
   void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack);

   void getAngularAcceleration(FixedFrameVector3DBasics angularAccelerationToPack);

   default void getAngularData(FixedFrameQuaternionBasics orientationToPack, FixedFrameVector3DBasics angularVelocityToPack,
                                      FixedFrameVector3DBasics angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}