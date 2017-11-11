package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.robotics.screwTheory.RigidBody;

public class WholeBodyTrajectoryToolboxMessageTools
{
   public static interface FunctionTrajectory
   {
      public Pose3D compute(double time);
   }

   public static FunctionTrajectory createFunctionTrajectory(WaypointBasedTrajectoryMessage message)
   {
      return new FunctionTrajectory()
      {
         @Override
         public Pose3D compute(double time)
         {
            Pose3D current = new Pose3D();

            Pose3D previous = null;
            Pose3D next = null;
            double t0 = Double.NaN;
            double tf = Double.NaN;

            for (int i = 1; i < message.getNumberOfWaypoints(); i++)
            {
               t0 = message.getWaypointTime(i - 1);
               tf = message.getWaypointTime(i);
               previous = message.getWaypoint(i - 1);
               next = message.getWaypoint(i);
               if (time < message.getWaypointTime(i))
                  break;
            }

            double alpha = (time - t0) / (tf - t0);
            alpha = MathTools.clamp(alpha, 0.0, 1.0);
            current.interpolate(previous, next, alpha);

            return current;
         }
      };
   }

   public static WaypointBasedTrajectoryMessage createTrajectoryMessage(RigidBody endEffector, double t0, double tf, double timeResolution,
                                                                        FunctionTrajectory trajectoryToDiscretize,
                                                                        ConfigurationSpaceName... unconstrainedDegreesOfFreedom)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      // Adjust the timeResolution using the numberOfWaypoints:
      timeResolution = (tf - t0) / (numberOfWaypoints - 1);

      double[] waypointTimes = new double[numberOfWaypoints];
      Pose3D[] waypoints = new Pose3D[numberOfWaypoints];

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double waypointTime = i * timeResolution + t0;

         waypointTimes[i] = waypointTime;
         waypoints[i] = trajectoryToDiscretize.compute(waypointTime);
      }

      return new WaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints, unconstrainedDegreesOfFreedom);
   }
}
