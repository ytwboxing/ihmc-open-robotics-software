package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class ExploreFootstepDataCommand
{
   private static void GenerateFootstepDataMessage()
   {
      //The swing trajectory has two default waypoints, the startPoint(set to current foot state at lift-off) and the 
      //final point defined by the location and orientation
      controller_msgs.msg.dds.FootstepDataMessage message = new controller_msgs.msg.dds.FootstepDataMessage();
      message.sequence_id_ = 1; //ID of footstep
      message.robot_side_ = 0; //0-Left 1-Right
      message.location_.set(new us.ihmc.euclid.tuple3D.Point3D(0, 0, 0)); //Point3D that gives position of the footstep (sole frame) in world frame.
      message.orientation_.set(0, 0, 0, 1); // quaterion entered (x,y,z,s) where x,y,z is the vector part, s is scalar
      //message.predicted_contact_points_2d_ //vertices of expected contact polygon btwn foot and world
      //leave empty to use default support polygon
      message.trajectory_type_ = us.ihmc.robotics.trajectories.TrajectoryType.DEFAULT.toByte(); //What swing trajectory should be, recommended is default
      message.swing_height_ = -1.0; //how high the robot should swing its foot, (Setting less than 0.0 will initiate default value

      // Can define two additional waypoints to shape the swing trajectory
      //message.custom_position_waypoints_ = ; 
      // The percentages along the trajectory that the waypoints are. If value is empty, sets default. 
      message.custom_waypoint_proportions_ = new us.ihmc.idl.IDLSequence.Double(1, "type_6");

      /*
       * if TRAJECTORY_TYPE_WAYPOINTS then there will be a list of waypoints for the swing foot to follow
       * if the expected_initial_location and expected_initial_orientation are filled then the
       * swing_trajectory_blend_duration_ defines the length of time from the beginning of the swing phase
       * that the trajectory will be altered to account for the error btwn the actual beginning state and
       * expected
       */
      //message.swing_trajectory_ = ; //list of waypoints for the trajectory
      message.swing_trajectory_blend_duration_ = 0;

      message.swing_duration_ = -1.0; //The swingDuration is the time a foot is not in ground contact during a step (non-positive means default will be used)
      message.transfer_duration_ = -1.0; //The transferDuration is the time spent with the feet in ground contact before a step (non-positive means default will be used)
      message.execution_delay_time_ = 0; //The time to delay this command on the controller side before being executed.
      message.swing_duration_shift_fraction_ = -1.0; //fraction of the swing duration spent shifting the weight from the heel to the toe (remain at toe after)
      message.swing_split_fraction_ = -1.0; //fraction of the shift portion of swing duration spent shifting the weight from the heel of the foot to the ball of the foot.
      message.transfer_split_fraction_ = -1.0; //fraction of the transfer duration spent shifting the weight from the trailing foot to the middle of the stance.
      message.transfer_weight_distribution_ = -1.0; //fraction through transfer that the CoP midpoint is located at (lower means at trailing foot, higher means at leading)
      message.touchdown_duration_ = -1.0; //Time spent after touchdown to transition from heel or toe support to full foot support.
      message.liftoff_duration_ = -1.0; //Time spent in toe or heel support before the step. This duration is part of the transfer duration
   }

   private static void GenerateFootstepDataCommand(controller_msgs.msg.dds.FootstepDataMessage message)
   {
      us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand command = new us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand();
      //command.sequenceId = message.getSequenceId();
      command.setRobotSide(RobotSide.fromByte(message.getRobotSide()));
      command.setTrajectoryType(TrajectoryType.fromByte(message.getTrajectoryType()));
      command.setSwingHeight(message.getSwingHeight());
      command.setSwingTrajectoryBlendDuration(message.getSwingTrajectoryBlendDuration());
      command.setPose(message.getLocation(), message.getOrientation()); //Worldframe set at initiation of Pose within footstep object
      //command.setPredictedContactPoints(message.getPredictedContactPoints2d());

   }
}
