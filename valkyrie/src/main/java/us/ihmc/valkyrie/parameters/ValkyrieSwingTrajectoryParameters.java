package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class ValkyrieSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   private final RobotTarget target;
   private final ValkyriePhysicalProperties physicalProperties;

   public ValkyrieSwingTrajectoryParameters(ValkyriePhysicalProperties physicalProperties, RobotTarget target)
   {
      this.physicalProperties = physicalProperties;
      this.target = target;
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }

   @Override
   public double getMinMechanicalLegLength()
   {
      return 0.1 * physicalProperties.getModelSizeScale();
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredTouchdownVelocity()
   {
      switch (target)
      {
      case SCS:
         return -0.3 * physicalProperties.getModelSizeScale();
      case GAZEBO:
      case REAL_ROBOT:
      default:
         return -0.1 * physicalProperties.getModelSizeScale();
      }
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredTouchdownAcceleration()
   {
      switch(target)
      {
      case REAL_ROBOT:
      case GAZEBO:
         return -1.0 * physicalProperties.getModelSizeScale();
      default:
         return -2.0 * physicalProperties.getModelSizeScale();
      }
   }

}
