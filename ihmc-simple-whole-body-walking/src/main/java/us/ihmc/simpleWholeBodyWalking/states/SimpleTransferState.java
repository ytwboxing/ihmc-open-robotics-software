package us.ihmc.simpleWholeBodyWalking.states;

<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
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
<<<<<<< HEAD
import us.ihmc.commons.MathTools;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
<<<<<<< HEAD
<<<<<<< HEAD
import us.ihmc.simpleWholeBodyWalking.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
=======
=======
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
>>>>>>> 2fb58d4d161... did the simple balance manager
import us.ihmc.simpleWholeBodyWalking.SimpleCenterOfMassHeightManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
<<<<<<< HEAD
import us.ihmc.yoVariables.variable.YoBoolean;
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager

public abstract class SimpleTransferState extends SimpleWalkingState
{
   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final SimpleCenterOfMassHeightManager comHeightManager;
<<<<<<< HEAD
<<<<<<< HEAD
   protected final SimpleBalanceManager balanceManager;
   protected final SimplePelvisOrientationManager pelvisOrientationManager;
=======
   protected final BalanceManager balanceManager;
=======
   protected final SimpleBalanceManager balanceManager;
>>>>>>> 2fb58d4d161... did the simple balance manager
   protected final PelvisOrientationManager pelvisOrientationManager;
>>>>>>> 13a03c33b98... set up the simple walking state controller
   protected final SimpleFeetManager feetManager;

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint3D desiredCoM = new FramePoint3D();

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   public SimpleTransferState(SimpleWalkingStateEnum transferStateEnum,
                              WalkingMessageHandler walkingMessageHandler,
                              HighLevelHumanoidControllerToolbox controllerToolbox,
                              SimpleControlManagerFactory managerFactory,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoVariableRegistry parentRegistry)
   {
      super(transferStateEnum, parentRegistry);
      this.transferToSide = transferStateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   @Override
   public void doAction(double timeInState)
   {
      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      //      comHeightManager.setSupportLeg(transferToSide.getOppositeSide());

      balanceManager.computeNormalizedEllipticICPError(transferToSide);
<<<<<<< HEAD
<<<<<<< HEAD
=======

      if (balanceManager.getNormalizedEllipticICPError() > balanceManager.getEllipticICPErrorForMomentumRecovery())
         balanceManager.setUseMomentumRecoveryModeForBalance(true);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
   }

   @Override
   public boolean isDone(double timeInState)
   {
<<<<<<< HEAD
<<<<<<< HEAD
      if (balanceManager.isICPPlanDone())
=======
      //If we're using a precomputed icp trajectory we can't rely on the icp planner's state to dictate when to exit transfer.
      boolean transferTimeElapsedUnderPrecomputedICPPlan = false;
      if (balanceManager.isPrecomputedICPPlannerActive())
      {
         transferTimeElapsedUnderPrecomputedICPPlan = timeInState > walkingMessageHandler.getNextTransferTime();
      }

      if (balanceManager.isICPPlanDone() || transferTimeElapsedUnderPrecomputedICPPlan)
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
      if (balanceManager.isICPPlanDone())
>>>>>>> 2fb58d4d161... did the simple balance manager
      {
         balanceManager.getCapturePoint(capturePoint2d);
         FrameConvexPolygon2DReadOnly supportPolygonInWorld = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();
         FrameConvexPolygon2DReadOnly nextPolygonInWorld = failureDetectionControlModule.getCombinedFootPolygonWithNextFootstep();

         double distanceToSupport = supportPolygonInWorld.distance(capturePoint2d);
         boolean isICPInsideNextSupportPolygon = nextPolygonInWorld.isPointInside(capturePoint2d);

         if (distanceToSupport > balanceManager.getICPDistanceOutsideSupportForStep() || (distanceToSupport > 0.0 && isICPInsideNextSupportPolygon))
            return true;
         else if (balanceManager.getNormalizedEllipticICPError() < 1.0)
            return true;
<<<<<<< HEAD
<<<<<<< HEAD
=======
         else
            balanceManager.setUseMomentumRecoveryModeForBalance(true);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
      }

      return false;
   }

   @Override
   public void onEntry()
   {
      updateICPPlan();

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekTiming(0, stepTiming);
      }
      else
      {
         stepTiming.setTimings(Double.NaN, Double.NaN);
      }

      double extraToeOffHeight = 0.0;
<<<<<<< HEAD
<<<<<<< HEAD
=======
      RobotSide swingSide = transferToSide.getOppositeSide();

      Footstep footstep = walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide);
      FixedFramePoint3DBasics transferFootPosition = footstep.getFootstepPose().getPosition();
      double transferTime = walkingMessageHandler.getNextTransferTime();
      comHeightManager.transfer(transferFootPosition, transferTime, swingSide, extraToeOffHeight);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager

      balanceManager.getFinalDesiredCoMPosition(desiredCoM);
      NewTransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(
            transferToSide);
      transferToAndNextFootstepsData.setComAtEndOfState(desiredCoM);
      comHeightManager.setSupportLeg(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   protected void updateICPPlan()
   {
      balanceManager.clearICPPlan();
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekFootstep(0, nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
<<<<<<< HEAD
<<<<<<< HEAD
=======
         balanceManager.setUpcomingFootstep(nextFootstep);
>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
>>>>>>> 2fb58d4d161... did the simple balance manager
      }
      else
      {
         failureDetectionControlModule.setNextFootstep(null);
<<<<<<< HEAD
<<<<<<< HEAD
      }

=======
         balanceManager.setUpcomingFootstep(null);
      }

      balanceManager.resetPushRecovery();

>>>>>>> 13a03c33b98... set up the simple walking state controller
=======
      }

>>>>>>> 2fb58d4d161... did the simple balance manager
      double transferTime = walkingMessageHandler.getNextTransferTime();
      pelvisOrientationManager.setTrajectoryTime(transferTime);
   }

   public boolean isInitialTransfer()
   {
      return getPreviousWalkingStateEnum() == SimpleWalkingStateEnum.STANDING;
   }
<<<<<<< HEAD
=======

   @Override
   public void onExit()
   {
      feetManager.reset();
   }
>>>>>>> 13a03c33b98... set up the simple walking state controller
}