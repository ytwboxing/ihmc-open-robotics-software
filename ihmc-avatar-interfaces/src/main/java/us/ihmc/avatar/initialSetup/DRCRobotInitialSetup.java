package us.ihmc.avatar.initialSetup;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.util.List;

public interface DRCRobotInitialSetup<T extends Robot>
{
   void initializeRobot(T robot, DRCRobotJointMap jointMap);

   default List<Double> getInitialJointAngles()
   {
      throw new RuntimeException("Not implemented.");
   }

   default Pose3DReadOnly getInitialPelvisPose()
   {
      throw new RuntimeException("Not implemented.");
   }

   default void initializeFullRobotModel(FullHumanoidRobotModel fullRobotModel)
   {
      OneDoFJointBasics[] allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      List<Double> initialJointAngles = getInitialJointAngles();
      for (int i = 0; i < initialJointAngles.size(); i++)
      {
         allJointsExcludingHands[i].setQ(initialJointAngles.get(i));
         allJointsExcludingHands[i].setQd(0.0);
      }

      fullRobotModel.getRootJoint().getJointPose().set(getInitialPelvisPose());
      fullRobotModel.getRootJoint().setJointVelocity(0, new DMatrixRMaj(6, 1));
      fullRobotModel.getRootJoint().getPredecessor().updateFramesRecursively();
   }

   void setInitialYaw(double yaw);
   double getInitialYaw();

   void setInitialGroundHeight(double groundHeight);
   double getInitialGroundHeight();

   void setOffset(Vector3D additionalOffset);
   void getOffset(Vector3D offsetToPack);

   /**
    * Indicates whether the robot can be reset to its initial sim configuration with the implementation of this
    * interface setup after the first time. This is deprecated since all initial setups should support this eventually.
    * An example of an initial setup that can be reused is the Atlas one.
    *
    * @return whether this initial setup will work more then once.
    */
   @Deprecated
   default boolean supportsReset()
   {
      return false;
   }
}
