package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.RhoValueObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoMMPCQPSolver
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("mpcSolverTimer", 0.5, registry);
   private final ActiveSetQPSolver qpSolver;

   private final YoBoolean addRateRegularization = new YoBoolean("AddRateRegularization", registry);

   final DMatrixRMaj solverInput_H;
   final DMatrixRMaj solverInput_f;

   private final DMatrixRMaj solverInput_H_previous;
   private final DMatrixRMaj solverInput_f_previous;

   final DMatrixRMaj solverInput_Aeq;
   final DMatrixRMaj solverInput_beq;
   final DMatrixRMaj solverInput_Ain;
   final DMatrixRMaj solverInput_bin;

   //   private final DMatrixRMaj solverInput_lb;
   //   private final DMatrixRMaj solverInput_ub;

   //   private final DMatrixRMaj solverInput_lb_previous;
   //   private final DMatrixRMaj solverInput_ub_previous;

   private final DMatrixRMaj solverOutput;

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);

   private final YoDouble comCoefficientRegularization = new YoDouble("comCoefficientRegularization", registry);
   private final YoDouble rhoCoefficientRegularization = new YoDouble("rhoCoefficientRegularization", registry);

   private final YoDouble comRateCoefficientRegularization = new YoDouble("comRateCoefficientRegularization", registry);
   private final YoDouble rhoRateCoefficientRegularization = new YoDouble("rhoRateCoefficientRegularization", registry);

   private int problemSize;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   final QPInputTypeA qpInput = new QPInputTypeA(0);

   private final MPCIndexHandler indexHandler;
   private final MPCQPInputCalculator inputCalculator;

   private final double dt;

   public CoMMPCQPSolver(MPCIndexHandler indexHandler, double dt, double gravityZ, YoRegistry parentRegistry)
   {
      this.indexHandler = indexHandler;
      this.dt = dt;

      comCoefficientRegularization.set(1e-5);
      rhoCoefficientRegularization.set(1e-5);

      rhoRateCoefficientRegularization.set(1e-6);
      comRateCoefficientRegularization.set(1e-6);

      qpSolver = new JavaQuadProgSolver();
      inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      int problemSize = 4 * 4 * 4 * 2 + 10;
      solverInput_H = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f = new DMatrixRMaj(problemSize, 1);

      solverInput_H_previous = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f_previous = new DMatrixRMaj(problemSize, 1);

      solverInput_Aeq = new DMatrixRMaj(0, problemSize);
      solverInput_beq = new DMatrixRMaj(0, 1);
      solverInput_Ain = new DMatrixRMaj(0, problemSize);
      solverInput_bin = new DMatrixRMaj(0, 1);

      //      solverInput_lb = new DMatrixRMaj(problemSize, 1);
      //      solverInput_ub = new DMatrixRMaj(problemSize, 1);

      //      solverInput_lb_previous = new DMatrixRMaj(problemSize, 1);
      //      solverInput_ub_previous = new DMatrixRMaj(problemSize, 1);

      //      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      //      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverOutput = new DMatrixRMaj(problemSize, 1);

      parentRegistry.addChild(registry);
   }

   public void setComCoefficientRegularizationWeight(double weight)
   {
      this.comCoefficientRegularization.set(weight);
   }

   public void setRhoCoefficientRegularizationWeight(double weight)
   {
      this.rhoCoefficientRegularization.set(weight);
   }

   public void setComRateCoefficientRegularizationWeight(double weight)
   {
      this.comRateCoefficientRegularization.set(weight);
   }

   public void setRhoRateCoefficientRegularizationWeight(double weight)
   {
      this.rhoRateCoefficientRegularization.set(weight);
   }

   public void setUseWarmStart(boolean useWarmStart)
   {
      this.useWarmStart = useWarmStart;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void notifyResetActiveSet()
   {
      this.resetActiveSet = true;
   }

   private boolean pollResetActiveSet()
   {
      boolean ret = resetActiveSet;
      resetActiveSet = false;
      return ret;
   }

   public void initialize()
   {
      int previousProblemSize = problemSize;
      problemSize = indexHandler.getTotalProblemSize();

      if (previousProblemSize != problemSize)
      //      if (true)
      {
         qpInput.setNumberOfVariables(problemSize);

         solverInput_H.reshape(problemSize, problemSize);
         solverInput_f.reshape(problemSize, 1);

         solverInput_H_previous.reshape(problemSize, problemSize);
         solverInput_f_previous.reshape(problemSize, 1);

         //         solverInput_lb.reshape(problemSize, 1);
         //         solverInput_ub.reshape(problemSize, 1);

         //         solverInput_lb_previous.reshape(problemSize, 1);
         //         solverInput_ub_previous.reshape(problemSize, 1);

         solverOutput.reshape(problemSize, 1);

         resetRateRegularization();
         notifyResetActiveSet();
      }

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);
      solverInput_Ain.reshape(0, problemSize);
      solverInput_bin.reshape(0, 1);

      solverInput_H.zero();
      solverInput_f.zero();

      //      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      //      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);
   }

   public void resetRateRegularization()
   {
      addRateRegularization.set(false);
   }

   private void addCoefficientRegularization()
   {
      addValueRegularization();

      if (addRateRegularization.getBooleanValue())
         addRateRegularization();
   }

   private void addValueRegularization()
   {
      int i = 0;
      for (; i < indexHandler.getRhoCoefficientStartIndex(0); i++)
         solverInput_H.add(i, i, comCoefficientRegularization.getDoubleValue());
      for (; i < problemSize; i++)
         solverInput_H.add(i, i, rhoCoefficientRegularization.getDoubleValue());
   }

   private void addRateRegularization()
   {
      double comCoefficientFactor = dt * dt / comRateCoefficientRegularization.getDoubleValue();
      double rhoCoefficientFactor = dt * dt / rhoRateCoefficientRegularization.getDoubleValue();
      int i = 0;
      for (; i < indexHandler.getRhoCoefficientStartIndex(0); i++)
      {
         solverInput_H.add(i, i, 1.0 / comCoefficientFactor);
         solverInput_f.add(i, 0, -solverOutput.get(i, 0) / comCoefficientFactor);
      }
      for (; i < problemSize; i++)
      {
         solverInput_H.add(i, i, 1.0 / rhoCoefficientFactor);
         solverInput_f.add(i, 0, -solverOutput.get(i, 0) / rhoCoefficientFactor);
      }
   }

   public void submitMPCCommandList(MPCCommandList commandList)
   {
      for (int i = 0; i < commandList.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = commandList.getCommand(i);

         switch (command.getCommandType())
         {
            case VALUE:
               submitMPCValueObjective((MPCValueCommand) command);
               break;
            case CONTINUITY:
               submitCoMContinuityObjective((CoMContinuityCommand) command);
               break;
            case LIST:
               submitMPCCommandList((MPCCommandList) command);
               break;
            case RHO_VALUE:
               submitRhoValueCommand((RhoValueObjectiveCommand) command);
               break;
            default:
               throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
         }
      }
   }

   public void submitRhoValueCommand(RhoValueObjectiveCommand command)
   {
      boolean success = inputCalculator.calculateRhoValueCommand(qpInput, command);
      if (success)
         addInput(qpInput);
   }

   public void submitMPCValueObjective(MPCValueCommand command)
   {
      boolean success = inputCalculator.calculateValueObjective(qpInput, command);
      if (success)
         addInput(qpInput);
   }

   public void submitCoMContinuityObjective(CoMContinuityCommand command)
   {
      boolean success = inputCalculator.calculateCoMContinuityObjective(qpInput, command);
      if (success)
         addInput(qpInput);
   }

   public void addInput(QPInputTypeA input)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
               addObjective(input.taskJacobian, input.taskObjective, input.getWeightScalar());
            else
               throw new IllegalArgumentException("Not yet implemented.");
            break;
         case EQUALITY:
            addEqualityConstraint(input.taskJacobian, input.taskObjective);
            break;
         case LEQ_INEQUALITY:
            addMotionLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
            break;
         case GEQ_INEQUALITY:
            addMotionGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addObjective(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double taskWeight)
   {
      addObjective(taskJacobian, taskObjective, taskWeight, problemSize, solverInput_H, solverInput_f);
   }

   public static void addObjective(DMatrixRMaj taskJacobian,
                                   DMatrixRMaj taskObjective,
                                   double taskWeight,
                                   int problemSize,
                                   DMatrixRMaj solverInput_H,
                                   DMatrixRMaj solverInput_f)
   {
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      MatrixTools.multAddBlockInner(taskWeight, taskJacobian, solverInput_H, 0, 0);

      // Compute: f += - J^T W Objective
      MatrixTools.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, solverInput_f, 0, 0);
   }

   public void addEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addEqualityConstraint(taskJacobian, taskObjective, problemSize, solverInput_Aeq, solverInput_beq);
   }

   public static void addEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, int problemSize, DMatrixRMaj solverInput_Aeq, DMatrixRMaj solverInput_beq)
   {
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }

      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps_DDRM.insert(taskJacobian, solverInput_Aeq, previousSize, 0);
      CommonOps_DDRM.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   public void addMotionLesserOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMotionLesserOrEqualInequalityConstraint(taskJacobian, taskObjective, problemSize, solverInput_Ain, solverInput_bin);
   }

   public static void addMotionLesserOrEqualInequalityConstraint(DMatrixRMaj taskJacobian,
                                                                 DMatrixRMaj taskObjective,
                                                                 int problemSize,
                                                                 DMatrixRMaj solverInput_Ain,
                                                                 DMatrixRMaj solverInput_bin)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, 1.0, problemSize, solverInput_Ain, solverInput_bin);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMotionGreaterOrEqualInequalityConstraint(taskJacobian, taskObjective, problemSize, solverInput_Ain, solverInput_bin);
   }

   public static void addMotionGreaterOrEqualInequalityConstraint(DMatrixRMaj taskJacobian,
                                                                  DMatrixRMaj taskObjective,
                                                                  int problemSize,
                                                                  DMatrixRMaj solverInput_Ain,
                                                                  DMatrixRMaj solverInput_bin)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, -1.0, problemSize, solverInput_Ain, solverInput_bin);
   }

   private static void addInequalityConstraintInternal(DMatrixRMaj taskJacobian,
                                                       DMatrixRMaj taskObjective,
                                                       double sign,
                                                       int problemSize,
                                                       DMatrixRMaj solverInput_Ain,
                                                       DMatrixRMaj solverInput_bin)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solverInput_bin.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Ain.reshape(previousSize + taskSize, problemSize, true);
      solverInput_bin.reshape(previousSize + taskSize, 1, true);

      MatrixTools.setMatrixBlock(solverInput_Ain, previousSize, 0, taskJacobian, 0, 0, taskSize, variables, sign);
      MatrixTools.setMatrixBlock(solverInput_bin, previousSize, 0, taskObjective, 0, 0, taskSize, 1, sign);
   }

   public boolean solve()
   {
      addCoefficientRegularization();

      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      numberOfActiveVariables.set(problemSize);

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f);
      //      qpSolver.setVariableBounds(solverInput_lb, solverInput_ub);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      numberOfIterations.set(qpSolver.solve(solverOutput));

      qpSolverTimer.stopMeasurement();

      if (MatrixTools.containsNaN(solverOutput))
      {
         addRateRegularization.set(false);
         numberOfIterations.set(-1);
         return false;
      }

      addRateRegularization.set(true);

      solverInput_H_previous.set(solverInput_H);
      solverInput_f_previous.set(solverInput_f);

      //      solverInput_lb_previous.set(solverInput_lb);
      //      solverInput_ub_previous.set(solverInput_ub);

      return true;
   }

   public DMatrixRMaj getSolution()
   {
      return solverOutput;
   }
}
