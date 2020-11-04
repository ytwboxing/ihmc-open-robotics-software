package us.ihmc.valkyrie.simulation;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidPositionControlledRobotSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.valkyrie.ValkyrieInitialSetupFactories;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyriePositionControlledRobotSimulationEndToEndTest extends HumanoidPositionControlledRobotSimulationEndToEndTest
{
   private final double dt = 8.0e-4;
   private ValkyrieRobotModel robotModel;

   public static Path scriptFolderPath()
   {
      Path folderPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      folderPath = Paths.get(folderPath.toFile().getParentFile().getAbsolutePath(),
                             "/ihmc-open-robotics-software/valkyrie/src/main/resources/multiContact/scripts");
      return folderPath;
   }

   @Override
   public ValkyrieRobotModel getRobotModel()
   {
      if (robotModel == null)
      {
         robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
         robotModel.setSimulationLowLevelControllerFactory(new ValkyrieSimulationLowLevelControllerFactory(robotModel.getJointMap(), dt));
         robotModel.setSimulateDT(dt);
         robotModel.setControllerDT(dt);
         robotModel.setEstimatorDT(dt);
      }
      return robotModel;
   }

   @Override
   protected DRCRobotModel getGhostRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }

   @Override
   protected HighLevelControllerParameters getPositionControlParameters(HighLevelControllerName positionControlState)
   {
      return new ValkyrieSimulationPositionControlParameters(getRobotModel().getHighLevelControllerParameters(),
                                                             getRobotModel().getJointMap(),
                                                             positionControlState);
   }

   @Test
   @Override
   public void testFreezeController(TestInfo testInfo) throws Exception
   {
      super.testFreezeController(testInfo);
   }

   @Test
   @Override
   public void testPositionController(TestInfo testInfo) throws Exception
   {
      super.testPositionController(testInfo);
   }

   @Test
   public void testCrawl1ToDabScript(TestInfo testInfo) throws Exception
   {
      runProcessedScriptTest(testInfo,
                             new File(scriptFolderPath().toFile(), "20200930_144631_Crawl1ToDab.json"),
                             ValkyrieInitialSetupFactories.newCrawl1(getRobotModel().getJointMap()),
                             new FlatGroundEnvironment());
   }
}
