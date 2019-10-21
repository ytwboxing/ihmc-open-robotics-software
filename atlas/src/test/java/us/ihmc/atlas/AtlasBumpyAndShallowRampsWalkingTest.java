package us.ihmc.atlas;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCBumpyAndShallowRampsWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasBumpyAndShallowRampsWalkingTest extends DRCBumpyAndShallowRampsWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
   
   @Tag("humanoid-obstacle-3")
   @Test
   @Override
   public void testDRCBumpyGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testDRCBumpyGroundWalking();
   }
   
   // This has never worked. Would be nice if we can get it to work.")
   @Tag("humanoid-obstacle-3")
   @Disabled
   @Test
   @Override
   public void testDRCOverRandomBlocks() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testDRCOverRandomBlocks();
   }
   
   @Tag("humanoid-obstacle-3")
   @Test
   @Override
   public void testDRCOverShallowRamp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testDRCOverShallowRamp();
   }
}
