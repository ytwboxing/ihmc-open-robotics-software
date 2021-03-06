package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;

public interface FrameSE3TrajectoryPointBasics
      extends SE3TrajectoryPointBasics, FrameSE3WaypointBasics, FrameEuclideanTrajectoryPointBasics, FrameSO3TrajectoryPointBasics
{
   default void set(double time, FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                    FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      set(position, orientation, linearVelocity, angularVelocity);
   }

   default void set(double time, FrameSE3WaypointBasics waypoint)
   {
      setTime(time);
      set(waypoint);
   }

   default void set(double time, FrameEuclideanWaypointBasics euclideanWaypoint, FrameSO3WaypointBasics so3Waypoint)
   {
      setTime(time);
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void setIncludingFrame(double time, FrameEuclideanWaypointBasics euclideanWaypoint, FrameSO3WaypointBasics so3Waypoint)
   {
      setTime(time);
      euclideanWaypoint.checkReferenceFrameMatch(so3Waypoint);
      setReferenceFrame(euclideanWaypoint.getReferenceFrame());
      set(euclideanWaypoint);
      set(so3Waypoint);
   }

   default void setIncludingFrame(double time, FrameSE3WaypointBasics waypoint)
   {
      setTime(time);
      setIncludingFrame(waypoint);
   }

   default void setIncludingFrame(double time, FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly linearVelocity,
                                  FrameVector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                  Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, position, orientation, linearVelocity, angularVelocity);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, SE3TrajectoryPointBasics trajectoryPoint)
   {
      setTime(trajectoryPoint.getTime());
      FrameSE3WaypointBasics.super.setIncludingFrame(referenceFrame, trajectoryPoint);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double time, SE3WaypointBasics waypoint)
   {
      setTime(time);
      setIncludingFrame(referenceFrame, waypoint);
   }

   default void setIncludingFrame(FrameSE3TrajectoryPointBasics other)
   {
      setTime(other.getTime());
      FrameSE3WaypointBasics.super.setIncludingFrame(other);
   }

   default void set(FrameSE3TrajectoryPointBasics other)
   {
      setTime(other.getTime());
      FrameSE3WaypointBasics.super.set(other);
   }

   default void getIncludingFrame(FrameSE3TrajectoryPointBasics otherToPack)
   {
      otherToPack.setIncludingFrame(this);
   }

   default void get(FrameSE3TrajectoryPointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default void getIncludingFrame(FrameEuclideanTrajectoryPointBasics euclideanTrajectoryPointToPack, FrameSO3TrajectoryPointBasics so3TrajectoryPointToPack)
   {
      getIncludingFrame(euclideanTrajectoryPointToPack);
      getIncludingFrame(so3TrajectoryPointToPack);
   }

   default void get(FrameEuclideanTrajectoryPointBasics euclideanTrajectoryPointToPack, FrameSO3TrajectoryPointBasics so3TrajectoryPointToPack)
   {
      get(euclideanTrajectoryPointToPack);
      get(so3TrajectoryPointToPack);
   }

   default boolean epsilonEquals(FrameSE3TrajectoryPointBasics other, double epsilon)
   {
      boolean timeEquals = EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon);
      return timeEquals && FrameSE3WaypointBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setTimeToNaN();
      FrameSE3WaypointBasics.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setTimeToZero();
      FrameSE3WaypointBasics.super.setToZero(referenceFrame);
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      SE3TrajectoryPointBasics.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      SE3TrajectoryPointBasics.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || SE3TrajectoryPointBasics.super.containsNaN();
   }
}
