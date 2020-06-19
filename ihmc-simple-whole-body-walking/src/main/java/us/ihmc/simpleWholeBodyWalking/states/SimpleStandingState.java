package us.ihmc.simpleWholeBodyWalking.states;

<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
=======
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
<<<<<<< HEAD
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleCenterOfMassHeightManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimplePelvisOrientationManager;
<<<<<<< HEAD
=======
=======
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.simpleWholeBodyWalking.SimpleCenterOfMassHeightManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 5942e55c22c... got simple pelvis orietnation manager
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleStandingState extends SimpleWalkingState
{
   private final CommandInputManager commandInputManager;
   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final SimpleCenterOfMassHeightManager comHeightManager;
<<<<<<< HEAD
<<<<<<< HEAD
   private final SimpleBalanceManager balanceManager;
   private final SimplePelvisOrientationManager pelvisOrientationManager;
=======
   private final BalanceManager balanceManager;
=======
   private final SimpleBalanceManager balanceManager;
<<<<<<< HEAD
>>>>>>> 2fb58d4d161... did the simple balance manager
   private final PelvisOrientationManager pelvisOrientationManager;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
   private final SimplePelvisOrientationManager pelvisOrientationManager;
>>>>>>> 5942e55c22c... got simple pelvis orietnation manager
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   public SimpleStandingState(CommandInputManager commandInputManager, WalkingMessageHandler walkingMessageHandler,
                              HighLevelHumanoidControllerToolbox controllerToolbox, SimpleControlManagerFactory managerFactory,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoVariableRegistry parentRegistry)
   {
      super(SimpleWalkingStateEnum.STANDING, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      RigidBodyBasics chest = controllerToolbox.getFullRobotModel().getChest();
      if (chest != null)
      {
         ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
            if (hand != null)
            {
               ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
               RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
               handManagers.put(robotSide, handManager);
            }
         }
      }

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      comHeightManager.setSupportLeg(RobotSide.LEFT);
   }

   @Override
   public void onEntry()
   {
      commandInputManager.clearAllCommands();

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

<<<<<<< HEAD
<<<<<<< HEAD
=======
      balanceManager.resetPushRecovery();
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
      balanceManager.enablePelvisXYControl();
      balanceManager.setICPPlanTransferFromSide(null);
      balanceManager.initializeICPPlanForStanding();

      walkingMessageHandler.reportWalkingComplete();

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initializeStanding();

      failureDetectionControlModule.setNextFootstep(null);
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);

   }

   @Override
   public void onExit()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).prepareForLocomotion();
      }

      if (pelvisOrientationManager != null)
      {
         pelvisOrientationManager.prepareForLocomotion(walkingMessageHandler.getNextStepTime());
      }

      balanceManager.disablePelvisXYControl();
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
   }

   @Override
   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return true;
   }

   @Override
   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return true;
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }
}