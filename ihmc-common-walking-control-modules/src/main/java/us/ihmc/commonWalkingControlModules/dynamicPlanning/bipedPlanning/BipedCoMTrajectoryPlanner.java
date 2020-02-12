package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

/**
 * This is a base class for bipeds for dynamic trajectory planning. It is used to generate feasible DCM, CoM, and VRP trajectories. The inputs to this class
 * are a list of {@link BipedTimedStep}, which are converted to a list of {@link ContactStateProvider}, which is then used by the {@link CoMTrajectoryPlanner}.
 * This is done using {@link BipedContactSequenceUpdater} class.
 *
 * <p>
 * WARNING: This class current generates garbage from the {@link BipedContactSequenceTools#computeStepTransitionsFromStepSequence}.
 * </p>
 */
public class BipedCoMTrajectoryPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedContactSequenceUpdater sequenceUpdater;
   private final CoMTrajectoryProvider comTrajectoryPlanner;

   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   @Deprecated
   private final List<BipedTimedStep> stepSequence = new ArrayList<>();

   public BipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleFrames, double gravityZ, double nominalCoMHeight,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      sequenceUpdater = new BipedContactSequenceUpdater(soleFrames, registry, yoGraphicsListRegistry);
      comTrajectoryPlanner = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);
      ((CoMTrajectoryPlanner) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

      parentRegistry.addChild(registry);
   }

   @Deprecated
   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void setNominalHeight(double nominalHeight)
   {
      comTrajectoryPlanner.setNominalCoMHeight(nominalHeight);
   }

   @Deprecated
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

   @Deprecated
   public void computeSetpoints(double currentTime, List<RobotSide> currentFeetInContact)
   {
      computeSetpoints(currentTime, currentFeetInContact, stepSequence);
   }

   public void computeSetpoints(double currentTime, List<RobotSide> currentFeetInContact, List<BipedTimedStep> stepSequence)
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

   public List<Trajectory3D> getVRPTrajectories()
   {
      return comTrajectoryPlanner.getVRPTrajectories();
   }

}
