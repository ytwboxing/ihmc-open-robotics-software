package us.ihmc.atlas;

<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasLegConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
=======
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasLegConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
>>>>>>> b667657d61a... Overrode some custom parameters
import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> b667657d61a... Overrode some custom parameters
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleWalkingControllerStateFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
<<<<<<< HEAD

import java.io.InputStream;
=======
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simpleWholeBodyWalking.SimpleWalkingControllerStateFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> b667657d61a... Overrode some custom parameters

import java.io.InputStream;

public class SimpleAtlasFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {

<<<<<<< HEAD
<<<<<<< HEAD
      DRCRobotModel model = getModel();
=======
      DRCRobotModel model = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                                RobotTarget.SCS,
<<<<<<< HEAD
                                                false);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
                                                false)
      {
         private static final String parameterFile = "/us/ihmc/atlas/parameters/experimental_controller_parameters.xml";

         @Override
         public String getParameterFileName()
         {
            return parameterFile;
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return null;
         }
      };
>>>>>>> 0564eaa6ee0... Adding parameter file for experimental walking state
=======
      DRCRobotModel model = getModel();
>>>>>>> b667657d61a... Overrode some custom parameters

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      new DRCFlatGroundWalkingTrack(robotInitialSetup,
                                    guiInitialSetup,
                                    scsInitialSetup,
                                    useVelocityAndHeadingScript,
                                    cheatWithGroundHeightAtForFootstep,
                                    model,
<<<<<<< HEAD
<<<<<<< HEAD
                                    new SimpleWalkingControllerStateFactory(model.getCapturePointPlannerParameters()));
   }

   private static DRCRobotModel getModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                 RobotTarget.SCS,
                                 false)
      {
         private static final String parameterFile = "/us/ihmc/atlas/parameters/experimental_controller_parameters.xml";

         @Override
         public String getParameterFileName()
         {
            return parameterFile;
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return null;
         }

         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return SimpleAtlasFlatGroundWalkingTrack.getWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters());
         }
      };
   }

   private static WalkingControllerParameters getWalkingControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      return new AtlasWalkingControllerParameters(target, jointMap, contactPointParameters)
      {
         @Override
         public LegConfigurationParameters getLegConfigurationParameters()
         {
            return new AtlasLegConfigurationParameters(target == RobotTarget.REAL_ROBOT)
            {
               @Override
               public LegConfigurationGains getBentLegGains()
               {
                  LegConfigurationGains gains = super.getBentLegGains();
                  gains.setJointSpaceKp(200.0);
                  return gains;
               }
            };
         }
      };
   }
=======
                                    new SimpleWalkingControllerStateFactory());
=======
                                    new SimpleWalkingControllerStateFactory(model.getCapturePointPlannerParameters()));
>>>>>>> 4cde91fd214... Passed correct ICP parameters to customState manager.
   }

<<<<<<< HEAD
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
   private static DRCRobotModel getModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                 RobotTarget.SCS,
                                 false)
      {
         private static final String parameterFile = "/us/ihmc/atlas/parameters/experimental_controller_parameters.xml";

         @Override
         public String getParameterFileName()
         {
            return parameterFile;
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return null;
         }

         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return SimpleAtlasFlatGroundWalkingTrack.getWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters());
         }
      };
   }

   private static WalkingControllerParameters getWalkingControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      return new AtlasWalkingControllerParameters(target, jointMap, contactPointParameters)
      {
         @Override
         public LegConfigurationParameters getLegConfigurationParameters()
         {
            return new AtlasLegConfigurationParameters(target == RobotTarget.REAL_ROBOT)
            {
               @Override
               public LegConfigurationGains getBentLegGains()
               {
                  LegConfigurationGains gains = super.getBentLegGains();
                  gains.setJointSpaceKp(200.0);
                  return gains;
               }
            };
         }
      };
   }
>>>>>>> b667657d61a... Overrode some custom parameters
}
