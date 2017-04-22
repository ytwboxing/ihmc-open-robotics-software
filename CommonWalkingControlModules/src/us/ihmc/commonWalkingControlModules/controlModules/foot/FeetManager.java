package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.CentroidProjectionToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ICPPlanToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.WrapperForMultipleToeOffCalculators;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

import java.util.ArrayList;

public class FeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<FootControlModule> footControlModules = new SideDependentList<>();

   private final ToeOffCalculator toeOffCalculator;
   private final ToeOffManager toeOffManager;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<ReferenceFrame> soleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   // TODO Needs to be cleaned up someday... (Sylvain)
   public FeetManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      feet = controllerToolbox.getContactableFeet();

      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactStates.put(robotSide, controllerToolbox.getFootContactState(robotSide));

      ArrayList<ToeOffCalculator> toeOffCalculators = new ArrayList<>();
      toeOffCalculators.add(new CentroidProjectionToeOffCalculator(contactStates, feet, walkingControllerParameters, registry));
      toeOffCalculators.add(new ICPPlanToeOffCalculator(contactStates, feet, registry));
      toeOffCalculator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, registry);

      toeOffManager = new ToeOffManager(controllerToolbox, toeOffCalculator, walkingControllerParameters, feet, registry);

      this.footSwitches = controllerToolbox.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();

      YoSE3PIDGainsInterface swingFootControlGains = walkingControllerParameters.createSwingFootControlGains(registry);
      YoSE3PIDGainsInterface holdPositionFootControlGains = walkingControllerParameters.createHoldPositionFootControlGains(registry);
      YoSE3PIDGainsInterface toeOffFootControlGains = walkingControllerParameters.createToeOffFootControlGains(registry);
      YoSE3PIDGainsInterface edgeTouchdownFootControlGains = walkingControllerParameters.createEdgeTouchdownFootControlGains(registry);

      walkingControllerParameters.getOrCreateExplorationParameters(registry);
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = new FootControlModule(robotSide, toeOffCalculator, walkingControllerParameters, swingFootControlGains,
               holdPositionFootControlGains, toeOffFootControlGains, edgeTouchdownFootControlGains, controllerToolbox, registry);

         footControlModules.put(robotSide, footControlModule);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeights(double highFootWeight, double defaultFootWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(highFootWeight, defaultFootWeight);
      }
   }

   public void setWeights(Vector3D highAngularFootWeight, Vector3D highLinearFootWeight, Vector3D defaultAngularFootWeight, Vector3D defaultLinearFootWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(highAngularFootWeight, highLinearFootWeight, defaultAngularFootWeight, defaultLinearFootWeight);
      }
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).hasFootHitGround(); //debug
         footControlModules.get(robotSide).doControl();
      }
   }

   public void requestSwing(RobotSide upcomingSwingSide, Footstep footstep, double swingTime)
   {
      if (!footstep.getTrustHeight())
      {
         FramePoint supportSolePosition = new FramePoint(soleZUpFrames.get(upcomingSwingSide.getOppositeSide()));
         supportSolePosition.changeFrame(footstep.getFootstepPose().getReferenceFrame());
         double newHeight = supportSolePosition.getZ();
         footstep.setZ(newHeight);
      }

      FootControlModule footControlModule = footControlModules.get(upcomingSwingSide);
      footControlModule.setFootstep(footstep, swingTime);
      setContactStateForSwing(upcomingSwingSide);
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      RobotSide robotSide = command.getRobotSide();
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.handleFootTrajectoryCommand(command);

      if (footControlModule.getCurrentConstraintType() != ConstraintType.MOVE_VIA_WAYPOINTS)
         setContactStateForMoveViaWaypoints(robotSide);
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public void replanSwingTrajectory(RobotSide swingSide, Footstep footstep, double swingTime, boolean continuousReplan)
   {
      footControlModules.get(swingSide).replanTrajectory(footstep, swingTime, continuousReplan);
   }

   public void requestMoveStraightTouchdownForDisturbanceRecovery(RobotSide swingSide)
   {
      footControlModules.get(swingSide).requestTouchdownForDisturbanceRecovery();
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (!command.isStopAllTrajectory())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.requestStopTrajectoryIfPossible();
      }
   }

   public boolean isInFlatSupportState(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInFlatSupportState();
   }

   public void correctCoMHeight(FrameVector2d desiredICPVelocity, double zCurrent, CoMHeightTimeDerivativesData comHeightData)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).updateLegSingularityModule();
      }

      // Correct, if necessary, the CoM height trajectory to avoid straight knee
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(desiredICPVelocity, comHeightData, zCurrent, pelvisZUpFrame);
      }

      // Do that after to make sure the swing foot will land
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).correctCoMHeightTrajectoryForUnreachableFootStep(comHeightData);
      }
   }

   public void initializeContactStatesForDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide.getOppositeSide()) == ConstraintType.SWING) // That case happens when doing 2 steps on same side
            setFlatFootContactState(transferToSide.getOppositeSide());
         setFlatFootContactState(transferToSide); // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
      }

      reset();
   }

   public void updateContactStatesInDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (getCurrentConstraintType(robotSide) == ConstraintType.TOES)
               setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide) == ConstraintType.TOES)
            setFlatFootContactState(transferToSide);
      }
   }

   private final FrameVector footNormalContactVector = new FrameVector(worldFrame, 0.0, 0.0, 1.0);

   public void setOnToesContactState(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      if (footControlModule.isInFlatSupportState())
      {
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
         footNormalContactVector.changeFrame(worldFrame);
      }
      else
      {
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      }

      footControlModule.setContactState(ConstraintType.TOES, footNormalContactVector);
   }

   public void setFlatFootContactState(RobotSide robotSide)
   {
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);

      if (footControlModules.get(robotSide).getCurrentConstraintType() == ConstraintType.TOES)
         controllerToolbox.restorePreviousFootContactPoints(robotSide);

      FootControlModule supportFootControlModule = footControlModules.get(robotSide.getOppositeSide());
      supportFootControlModule.setAllowFootholdAdjustments(true);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);

      FootControlModule supportFootControlModule = footControlModules.get(robotSide.getOppositeSide());
      supportFootControlModule.setAllowFootholdAdjustments(false);
   }

   private void setContactStateForMoveViaWaypoints(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.MOVE_VIA_WAYPOINTS);
   }

   public ToeOffManager getToeOffManager()
   {
      return toeOffManager;
   }

   public boolean willDoToeOffDoubleSupport(Footstep nextFootstep, RobotSide transferToSide)
   {
      return toeOffManager.canDoToeOffDoubleSupport(nextFootstep, transferToSide);
   }

   public boolean willDoToeOffSingleSupport(Footstep nextFootstep, RobotSide transferToSide)
   {
      return toeOffManager.canDoToeOffSingleSupoprt(nextFootstep, transferToSide);
   }

   /**
    * {@link ToeOffManager#updateToeOffStatusSingleSupport(FramePoint, FramePoint2d, FramePoint2d, FramePoint2d)}
    * @return {@link ToeOffManager#doLineToeOff} or {@link ToeOffManager#doPointToeOff()}
    */
   public void updateToeOffSingleSupport(Footstep nextFootstep, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d currentICP, FramePoint2d desiredICP)
   {
      toeOffManager.inSingleSupport();
      toeOffManager.submitNextFootstep(nextFootstep);
      toeOffManager.updateToeOffStatusSingleSupport(exitCMP, desiredECMP, desiredICP, currentICP);
   }

   /**
    * {@link ToeOffManager#updateToeOffStatusDoubleSupport(RobotSide, FramePoint, FramePoint2d, FramePoint2d, FramePoint2d)}
    * @return {@link ToeOffManager#doLineToeOff} or {@link ToeOffManager#doPointToeOff()}
    */
   public void updateToeOffDoubleSupport(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      toeOffManager.inDoubleSupport();
      toeOffManager.updateToeOffStatusDoubleSupport(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);
   }

   public boolean willDoPointToeOff()
   {
      return toeOffManager.doPointToeOff();
   }

   public boolean willDoLineToeOff()
   {
      return toeOffManager.doLineToeOff();
   }

   public void useToeOffPointContact(RobotSide trailingLeg)
   {
      footControlModules.get(trailingLeg).setUsePointContactInToeOff(true);
   }

   public void useToeOffLineContact(RobotSide trailingLeg)
   {
      footControlModules.get(trailingLeg).setUsePointContactInToeOff(false);
   }

   public void requestToeOff(RobotSide trailingLeg)
   {
      if (footControlModules.get(trailingLeg).isInToeOff())
         return;
      setOnToesContactState(trailingLeg);
   }

   public void computeToeOffContactLine(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredCMP)
   {
      toeOffCalculator.setExitCMP(exitCMP, trailingLeg);
      toeOffCalculator.computeToeOffContactLine(desiredCMP, trailingLeg);
   }

   public void computeToeOffContactPoint(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredCMP)
   {
      toeOffCalculator.setExitCMP(exitCMP, trailingLeg);
      toeOffCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
   }

   public boolean shouldComputeToeLineContact()
   {
      return toeOffManager.shouldComputeToeLineContact();
   }

   public boolean shouldComputeToePointContact()
   {
      return toeOffManager.shouldComputeToePointContact();
   }

   public void reset()
   {
      toeOffManager.reset();
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).resetHeightCorrectionParametersForSingularityAvoidance();
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(RobotSide robotSide, double speedUpFactor)
   {
      return footControlModules.get(robotSide).requestSwingSpeedUp(speedUpFactor);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotSide).createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }
      return ret;
   }

   public void initializeFootExploration(RobotSide robotSideToExplore)
   {
      if (robotSideToExplore == null) return;
      FootControlModule footControlModule = footControlModules.get(robotSideToExplore);
      footControlModule.initializeFootExploration();
   }
}
