package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner_MultipleeCMPs;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class BipedCoMTrajectoryPlanner_MultipleeCMPs
{
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedContactSequenceUpdater sequenceUpdater;
   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;

   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   private final List<BipedTimedStep> stepSequence = new ArrayList<>();
   
   // constructs CoMTrajectoryPlanner w/ Multiple eCMPs for each contact point
   public BipedCoMTrajectoryPlanner_MultipleeCMPs(SideDependentList<MovingReferenceFrame> soleFrames, double gravityZ, double nominalCoMHeight,
                                                  YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry) {
      
      sequenceUpdater = new BipedContactSequenceUpdater(soleFrames, registry, yoGraphicsListRegistry);
      comTrajectoryPlanner = new CoMTrajectoryPlanner_MultipleeCMPs(gravityZ, nominalCoMHeight, registry); // construct new one
      ((CoMTrajectoryPlanner_MultipleeCMPs) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

      parentRegistry.addChild(registry);
   }
   
   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void addStepToSequence(BipedTimedStep step)
   {
      stepSequence.add(step);
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void initialize()
   {
      sequenceUpdater.initialize();
   }

   void computeSetpoints(double currentTime, List<RobotSide> currentFeetInContact)
   {
      sequenceUpdater.update(stepSequence, currentFeetInContact, currentTime);

      double timeInPhase = currentTime - sequenceUpdater.getAbsoluteContactSequence().get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);     
      
      comTrajectoryPlanner.solveForTrajectory(sequenceUpdater.getContactSequence());
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return comTrajectoryPlanner.getDesiredDCMPosition();
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return comTrajectoryPlanner.getDesiredDCMVelocity();
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return comTrajectoryPlanner.getDesiredCoMPosition();
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return comTrajectoryPlanner.getDesiredCoMVelocity();
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return comTrajectoryPlanner.getDesiredCoMAcceleration();
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }
   
   public FramePoint3DReadOnly getDesiredECMPPosition_left()
   {
      return ((CoMTrajectoryPlanner_MultipleeCMPs) comTrajectoryPlanner).getDesiredECMPPosition_left();
   }
   
   public FramePoint3DReadOnly getDesiredECMPPosition_right()
   {
      return ((CoMTrajectoryPlanner_MultipleeCMPs) comTrajectoryPlanner).getDesiredECMPPosition_right();
   }
}
