package us.ihmc.simpleWholeBodyWalking;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
=======
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
>>>>>>> 29fb07b96d9... Added the set ICP parameters to SimpleWalkingControllerStateFactory.
=======
//import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
>>>>>>> 4cde91fd214... Passed correct ICP parameters to customState manager.
=======
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
>>>>>>> 9f1b5ce312b... Got Atlas to stand
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class SimpleWalkingControllerStateFactory implements HighLevelControllerStateFactory
{
   private SimpleWalkingControllerState walkingControllerState;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 4cde91fd214... Passed correct ICP parameters to customState manager.
   private SimpleControlManagerFactory managerFactory;
   private ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   
   public SimpleWalkingControllerStateFactory(ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters) {
<<<<<<< HEAD
      this.capturePointPlannerParameters = capturePointPlannerParameters; // Needed to pass ICPWithTimeFreezingPlannerParameters this way
   }
   
=======
   private final SimpleControlManagerFactory managerFactory;

   public SimpleWalkingControllerStateFactory(SimpleControlManagerFactory managerFactory)
   {
      this.managerFactory = managerFactory;
   }
=======
>>>>>>> dbc5c5a55f9... fixed compile

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
      this.capturePointPlannerParameters = capturePointPlannerParameters;
   }
   
>>>>>>> 4cde91fd214... Passed correct ICP parameters to customState manager.
   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (walkingControllerState == null)
      {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         managerFactory = new SimpleControlManagerFactory(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoVariableRegistry());
         managerFactory.setHighLevelHumanoidControllerToolbox(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());
         managerFactory.setWalkingControllerParameters(controllerFactoryHelper.getWalkingControllerParameters());
         managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
         
=======
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
         SimpleControlManagerFactory managerFactory = new SimpleControlManagerFactory(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoVariableRegistry());
>>>>>>> dbc5c5a55f9... fixed compile
=======
         
         SimpleControlManagerFactory managerFactory = new SimpleControlManagerFactory(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoVariableRegistry());
         managerFactory.setHighLevelHumanoidControllerToolbox(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());
         managerFactory.setWalkingControllerParameters(controllerFactoryHelper.getWalkingControllerParameters());
<<<<<<< HEAD
         
>>>>>>> 017fc976122... Fixed 2 bugs in running SimpleAtlasFGWT
=======
         managerFactory.setCapturePointPlannerParameters((ICPWithTimeFreezingPlannerParameters) controllerFactoryHelper.getIcpPlannerParameters());
>>>>>>> 29fb07b96d9... Added the set ICP parameters to SimpleWalkingControllerStateFactory.
=======
         managerFactory = new SimpleControlManagerFactory(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoVariableRegistry());
         managerFactory.setHighLevelHumanoidControllerToolbox(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());
         managerFactory.setWalkingControllerParameters(controllerFactoryHelper.getWalkingControllerParameters());
         managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
         
>>>>>>> 4cde91fd214... Passed correct ICP parameters to customState manager.
         walkingControllerState = new SimpleWalkingControllerState(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getStatusMessageOutputManager(),
                                                                   managerFactory, controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                   controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                   controllerFactoryHelper.getWalkingControllerParameters());
      }

      return walkingControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.WALKING;
   }
}
