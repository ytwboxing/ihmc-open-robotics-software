package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class SimpleLIPMDDPCalculator
{
   private final DiscreteHybridDynamics<LIPMState> dynamics;

   private double deltaT;
   private double modifiedDeltaT;

   private final RecyclingArrayList<DenseMatrix64F> desiredStateVector;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlVector;

   private int numberOfTimeSteps;

   private final DDPSolver<LIPMState> ddpSolver;

   public SimpleLIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new SimpleLIPMDynamics(deltaT, 1.0, gravityZ);
      LQCostFunction costFunction = new SimpleLIPMSimpleCostFunction(); // discrete, so we need to take that into account
      LQCostFunction terminalCostFunction = new SimpleLIPMTerminalCostFunction();
      this.deltaT = deltaT;

      ddpSolver = new DDPSolver<>(dynamics, costFunction, terminalCostFunction, true);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);

      desiredStateVector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      desiredControlVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      desiredStateVector.clear();
      desiredControlVector.clear();
   }

   public void setLineSearchGain(double lineSearchGain)
   {
      ddpSolver.setLineSearchGain(lineSearchGain);
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
      this.modifiedDeltaT = deltaT;
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public void initialize(DenseMatrix64F currentState, DenseMatrix64F currentControl, SegmentedFrameTrajectory3D copDesiredPlan)
   {
      modifiedDeltaT = computeDeltaT(copDesiredPlan.getFinalTime());
      dynamics.setTimeStepSize(modifiedDeltaT);

      desiredControlVector.clear();
      desiredStateVector.clear();

      double time = 0.0;
      copDesiredPlan.update(time, tempPoint, tempVector);
      DenseMatrix64F desiredState = desiredStateVector.add();

      desiredState.set(0, tempPoint.getX());
      desiredState.set(1, tempPoint.getY());
      desiredState.set(2, tempVector.getX());
      desiredState.set(3, tempVector.getY());

      DenseMatrix64F desiredControl = desiredControlVector.add();
      desiredControl.set(0, tempPoint.getX());
      desiredControl.set(1, tempPoint.getY());

      time += modifiedDeltaT;

      for (int i = 1; i < numberOfTimeSteps; i++)
      {
         copDesiredPlan.update(time, tempPoint, tempVector);
         desiredState = desiredStateVector.add();

         desiredState.set(0, tempPoint.getX());
         desiredState.set(1, tempPoint.getY());
         desiredState.set(2, tempVector.getX());
         desiredState.set(3, tempVector.getY());

         desiredControl = desiredControlVector.add();
         desiredControl.set(0, tempPoint.getX());
         desiredControl.set(1, tempPoint.getY());

         time += modifiedDeltaT;
      }

      ddpSolver.setDesiredTrajectories(desiredStateVector, desiredControlVector, currentState, currentControl, copDesiredPlan.getFinalTime());
      ddpSolver.solveBackwardLQRPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      ddpSolver.solveForwardLQRPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      ddpSolver.initializeDDPWithLQRSolution();
   }

   public void backwardDDPPass()
   {
      ddpSolver.solveBackwardDDPPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
   }

   public void forwardDDPPass()
   {
      ddpSolver.solveForwardDDPPass(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
   }

   private double computeDeltaT(double trajectoryLength)
   {
      numberOfTimeSteps = (int) Math.floor(trajectoryLength / deltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

   public double getDT()
   {
      return modifiedDeltaT;
   }

   public RecyclingArrayList<DenseMatrix64F> getControlVector()
   {
      return ddpSolver.getControlVector();
   }

   public RecyclingArrayList<DenseMatrix64F> getStateVector()
   {
      return ddpSolver.getStateVector();
   }
}
