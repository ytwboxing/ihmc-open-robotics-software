package us.ihmc.simpleWholeBodyWalking.states;

<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
=======
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleCenterOfMassHeightManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
<<<<<<< HEAD
import us.ihmc.yoVariables.variable.YoBoolean;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleTransferToStandingState extends SimpleWalkingState
{
<<<<<<< HEAD
<<<<<<< HEAD
=======
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
   private final YoDouble maxICPErrorToSwitchToStanding = new YoDouble("maxICPErrorToSwitchToStanding", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final SimpleCenterOfMassHeightManager comHeightManager;
<<<<<<< HEAD
<<<<<<< HEAD
   private final SimpleBalanceManager balanceManager;
   private final SimplePelvisOrientationManager pelvisOrientationManager;
   private final SimpleFeetManager feetManager;

=======
   private final BalanceManager balanceManager;
=======
   private final SimpleBalanceManager balanceManager;
>>>>>>> 2fb58d4d161... did the simple balance manager
   private final PelvisOrientationManager pelvisOrientationManager;
   private final SimpleFeetManager feetManager;

   private final Point3D midFootPosition = new Point3D();

>>>>>>> 13a03c33b98... set up the simple walking state controller
   public SimpleTransferToStandingState(WalkingMessageHandler walkingMessageHandler,
                                        HighLevelHumanoidControllerToolbox controllerToolbox,
                                        SimpleControlManagerFactory managerFactory,
                                        WalkingFailureDetectionControlModule failureDetectionControlModule,
                                        YoVariableRegistry parentRegistry)
   {
      super(SimpleWalkingStateEnum.TO_STANDING, parentRegistry);
      maxICPErrorToSwitchToStanding.set(0.025);

      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(RobotSide.LEFT);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (!balanceManager.isICPPlanDone())
         return false;

      return balanceManager.getICPErrorMagnitude() < maxICPErrorToSwitchToStanding.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();
<<<<<<< HEAD
<<<<<<< HEAD
=======
      balanceManager.resetPushRecovery();
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager

      SimpleWalkingStateEnum previousStateEnum = getPreviousWalkingStateEnum();

      // This can happen if walking is paused or aborted while the robot is on its toes already. In that case
      // restore the full foot contact.
      if (previousStateEnum != null && previousStateEnum.isDoubleSupport())
         feetManager.initializeContactStatesForDoubleSupport(null);

      RobotSide previousSupportSide = null;
      if (previousStateEnum != null)
      {
         if (previousStateEnum.getSupportSide() != null)
            previousSupportSide = previousStateEnum.getSupportSide();
         else if (previousStateEnum.getTransferToSide() != null)
            previousSupportSide = previousStateEnum.getTransferToSide();
      }

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(null);

      NewTransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(
            RobotSide.LEFT);
      double extraToeOffHeight = 0.0;
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();
<<<<<<< HEAD
<<<<<<< HEAD
=======
      Footstep footstepLeft = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
      Footstep footstepRight = walkingMessageHandler.getFootstepAtCurrentLocation(RobotSide.LEFT);
      midFootPosition.interpolate(footstepLeft.getFootstepPose().getPosition(), footstepRight.getFootstepPose().getPosition(), 0.5);
      comHeightManager.transfer(midFootPosition, finalTransferTime);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
      balanceManager.setICPPlanTransferFromSide(previousSupportSide);
      balanceManager.initializeICPPlanForTransferToStanding(finalTransferTime);
   }
<<<<<<< HEAD
<<<<<<< HEAD
=======

   @Override
   public void onExit()
   {
   }
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
}