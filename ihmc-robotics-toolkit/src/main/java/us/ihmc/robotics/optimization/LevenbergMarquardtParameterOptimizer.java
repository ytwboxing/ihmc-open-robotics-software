package us.ihmc.robotics.optimization;

<<<<<<< HEAD
<<<<<<< HEAD
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixDimensionException;
=======
=======
import java.util.function.Function;
>>>>>>> 2db4111f4b8... Defined input space and output space.
import java.util.function.UnaryOperator;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
>>>>>>> 2d0a07337e7... Replaced function output with UnaryOperator.

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class LevenbergMarquardtParameterOptimizer
{
   private static final boolean DEBUG = false;
   private int inputDimension;
   private int outputDimension;

   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction;
   private final UnaryOperator<DMatrixRMaj> outputCalculator;
<<<<<<< HEAD
<<<<<<< HEAD
   private boolean useDampingCoefficient = false; // TODO: add setter.
   private final DenseMatrix64F dampingCoefficient;
=======
   private boolean useDampingCoefficient = false;
   private final DMatrixRMaj dampingCoefficient;
>>>>>>> 14982396e4c... Corrected typo.
   private static final double DEFAULT_RESIDUAL_SCALER = 0.1;
   private double residualScaler = DEFAULT_RESIDUAL_SCALER;

   private final DenseMatrix64F currentInput;
   private final DenseMatrix64F currentOutput;
   private final DenseMatrix64F purterbationVector;
   private final DenseMatrix64F perturbedInput;
   private final DenseMatrix64F perturbedOutput;
   private final DenseMatrix64F jacobian;
=======

   private final DMatrixRMaj currentInput;
   private final OuputSpace currentOutputSpace;
   private final DMatrixRMaj purterbationVector;
   private final DMatrixRMaj perturbedInput;
   private final DMatrixRMaj perturbedOutput;
   private final DMatrixRMaj jacobian;
>>>>>>> 2db4111f4b8... Defined input space and output space.

<<<<<<< HEAD
   private final DenseMatrix64F jacobianTranspose;
   private final DenseMatrix64F squaredJacobian;
   private final DenseMatrix64F invMultJacobianTranspose;
=======
   private final DMatrixRMaj jacobianTranspose;
   private final DMatrixRMaj squaredJacobian;
   private final DMatrixRMaj dampingMatrix;
   private final DMatrixRMaj invMultJacobianTranspose;
>>>>>>> 5fd145d18c3... Enabled damping matrix.

   private final DenseMatrix64F optimizeDirection;

   /**
    * higher : make quick approach when the model and the data are in long distance. lower : slower
    * approach but better accuracy in near distance.
    */
   private double correspondenceThreshold = 1.0;
   private static final double DEFAULT_PERTURBATION = 0.0001;
   private static final double DEFAULT_DAMPING_COEFFICIENT = 0.001;
   private boolean useDamping = true;

   private double computationTime;
   private int iteration;
   private boolean optimized;

   /**
    * iterate the direction to minimize output space, -inv(J^T * J + W_N) * J^T * e
    */
   public LevenbergMarquardtParameterOptimizer(Function<DMatrixRMaj, RigidBodyTransform> inputFunction, UnaryOperator<DMatrixRMaj> outputCalculator,
                                               int inputParameterDimension, int outputDimension)
   {
      this.inputFunction = inputFunction;
      this.inputDimension = inputParameterDimension;
      this.outputDimension = outputDimension;
      this.outputCalculator = outputCalculator;

<<<<<<< HEAD
      dampingCoefficient = new DenseMatrix64F(inputParameterDimension, inputParameterDimension);

      currentInput = new DenseMatrix64F(inputParameterDimension, 1);
      currentOutput = new DenseMatrix64F(outputDimension, 1);
      purterbationVector = new DenseMatrix64F(inputParameterDimension, 1);
      perturbedInput = new DenseMatrix64F(inputParameterDimension, 1);
      perturbedOutput = new DenseMatrix64F(outputDimension, 1);
      jacobian = new DenseMatrix64F(outputDimension, inputParameterDimension);
=======
      currentInput = new DMatrixRMaj(inputParameterDimension, 1);
      currentOutputSpace = new OuputSpace(outputDimension);

      purterbationVector = new DMatrixRMaj(inputParameterDimension, 1);
      for (int i = 0; i < inputParameterDimension; i++)
         purterbationVector.set(i, DEFAULT_PERTURBATION);
      perturbedInput = new DMatrixRMaj(inputParameterDimension, 1);
      perturbedOutput = new DMatrixRMaj(outputDimension, 1);
      jacobian = new DMatrixRMaj(outputDimension, inputParameterDimension);
>>>>>>> 2db4111f4b8... Defined input space and output space.

      jacobianTranspose = new DenseMatrix64F(outputDimension, inputParameterDimension);
      squaredJacobian = new DenseMatrix64F(inputParameterDimension, inputParameterDimension);
      invMultJacobianTranspose = new DenseMatrix64F(inputParameterDimension, outputDimension);

<<<<<<< HEAD
<<<<<<< HEAD
      optimizeDirection = new DenseMatrix64F(inputParameterDimension, 1);
=======
      jacobianTranspose = new DMatrixRMaj(outputDimension, inputParameterDimension);
      squaredJacobian = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      dampingMatrix = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      invMultJacobianTranspose = new DMatrixRMaj(inputParameterDimension, outputDimension);
>>>>>>> 5fd145d18c3... Enabled damping matrix.

      correspondence = new boolean[outputDimension];
=======
      optimizeDirection = new DMatrixRMaj(inputParameterDimension, 1);
>>>>>>> 2db4111f4b8... Defined input space and output space.
   }

<<<<<<< HEAD
   public void reShape(int parameterDimension, int outputDimension)
   {
      this.parameterDimension = parameterDimension;
      this.outputDimension = outputDimension;

      dampingCoefficient.reshape(parameterDimension, parameterDimension);

      currentInput.reshape(parameterDimension, 1);
      currentOutput.reshape(outputDimension, 1);
      purterbationVector.reshape(parameterDimension, 1);
      perturbedInput.reshape(parameterDimension, 1);
      perturbedOutput.reshape(outputDimension, 1);
      jacobian.reshape(outputDimension, parameterDimension);

      jacobianTranspose.reshape(outputDimension, parameterDimension);
      squaredJacobian.reshape(parameterDimension, parameterDimension);
      invMultJacobianTranspose.reshape(parameterDimension, outputDimension);

      optimizeDirection.reshape(parameterDimension, 1);

      correspondence = new boolean[outputDimension];
   }

   public void setPerturbationVector(DenseMatrix64F purterbationVector)
=======
   public void setPerturbationVector(DMatrixRMaj purterbationVector)
>>>>>>> 2d0a07337e7... Replaced function output with UnaryOperator.
   {
      if (this.purterbationVector.getNumCols() != purterbationVector.getNumCols())
         throw new MatrixDimensionException("dimension is wrong. " + this.purterbationVector.getNumCols() + " " + purterbationVector.getNumCols());
      this.purterbationVector.set(purterbationVector);
   }

   public void setCorrespondenceThreshold(double correspondenceThreshold)
   {
      this.correspondenceThreshold = correspondenceThreshold;
   }

<<<<<<< HEAD
   private double computeQuality(DenseMatrix64F space, boolean[] correspondence)
   {
      double norm = 0.0;
      for (int i = 0; i < space.getNumRows(); i++)
      {
         if (correspondence[i])
         {
            norm = norm + space.get(i, 0) * space.get(i, 0);
         }
      }
      return norm;
   }

   private double computePureQuality(DenseMatrix64F space)
   {
      double norm = 0.0;
      for (int i = 0; i < space.getNumRows(); i++)
      {
         norm = norm + space.get(i, 0) * space.get(i, 0);
      }
      return norm;
   }

   private void updateDamping()
   {
      if (quality < initialQuality)
         residualScaler = quality / initialQuality * DEFAULT_RESIDUAL_SCALER;

      for (int i = 0; i < parameterDimension; i++)
      {
         for (int j = 0; j < parameterDimension; j++)
         {
            if (i == j)
            {
               dampingCoefficient.set(i, j, residualScaler);
            }
         }
      }
   }

   public void initialize()
=======
   public boolean initialize()
>>>>>>> 2db4111f4b8... Defined input space and output space.
   {
      iteration = 0;
      optimized = false;
      for (int i = 0; i < inputDimension; i++)
      {
         for (int j = 0; j < inputDimension; j++)
         {
            if (i == j)
            {
               dampingMatrix.set(i, j, DEFAULT_DAMPING_COEFFICIENT);
            }
         }
      }
      currentOutputSpace.updateOutputSpace(outputCalculator.apply(currentInput));

      return currentOutputSpace.computeCorrespondence();
   }

   public double iterate()
   {
      iteration++;
      long startTime = System.nanoTime();

<<<<<<< HEAD
      DenseMatrix64F newInput = new DenseMatrix64F(parameterDimension, 1);
=======
      DMatrixRMaj newInput = new DMatrixRMaj(inputDimension, 1);
>>>>>>> 2db4111f4b8... Defined input space and output space.

      currentOutputSpace.updateOutputSpace(outputCalculator.apply(currentInput));
      if (!currentOutputSpace.computeCorrespondence())
      {
         return -1;
      }
      currentOutputSpace.computeQuality();

      // start.      
      // compute jacobian.
      for (int i = 0; i < inputDimension; i++)
      {
         perturbedInput.set(currentInput);
         perturbedInput.add(i, 0, purterbationVector.get(i));

         perturbedOutput.set(outputCalculator.apply(perturbedInput));
         for (int j = 0; j < outputDimension; j++)
         {
            if (currentOutputSpace.isCorresponding(j))
            {
               double partialValue = (perturbedOutput.get(j) - currentOutputSpace.getOutput().get(j)) / purterbationVector.get(i);
               jacobian.set(j, i, partialValue);
            }
            else
            {
               jacobian.set(j, i, 0.0);
            }
         }
      }

      // compute direction.
      jacobianTranspose.set(jacobian);
      CommonOps.transpose(jacobianTranspose);

<<<<<<< HEAD
      CommonOps.mult(jacobianTranspose, jacobian, squaredJacobian);

      if (useDampingCoefficient)
      {
         updateDamping();
         CommonOps.add(squaredJacobian, dampingCoefficient, squaredJacobian);
      }
      CommonOps.invert(squaredJacobian);

      CommonOps.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);
      CommonOps.mult(invMultJacobianTranspose, currentOutput, optimizeDirection);
=======
      CommonOps_DDRM.mult(jacobianTranspose, jacobian, squaredJacobian);
      if (useDamping)
      {
         CommonOps_DDRM.add(squaredJacobian, dampingMatrix, squaredJacobian);
      }
      CommonOps_DDRM.invert(squaredJacobian);

      CommonOps_DDRM.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);
      CommonOps_DDRM.mult(invMultJacobianTranspose, currentOutputSpace.getOutput(), optimizeDirection);
>>>>>>> 2db4111f4b8... Defined input space and output space.

      // update currentInput.
      CommonOps.subtract(currentInput, optimizeDirection, newInput);

      // compute new quality.
      currentInput.set(newInput);

      double iterateTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
      if (DEBUG)
      {
         System.out.println("elapsed iteration time is " + iterateTime);
      }

      return currentOutputSpace.getCorrespondingQuality();
   }

   public void convertInputToTransform(DMatrixRMaj input, RigidBodyTransform transformToPack)
   {
      if (input.getData().length != inputDimension)
         throw new MatrixDimensionException("dimension is wrong. " + input.getData().length + " " + inputDimension);
      transformToPack.set(inputFunction.apply(input));
   }

   public int getNumberOfCorespondingPoints()
   {
      return currentOutputSpace.getNumberOfCorrespondingPoints();
   }

   public DenseMatrix64F getOptimalParameter()
   {
      return currentInput;
   }

   public boolean isSolved()
   {
      return optimized;
   }

   public double getQuality()
   {
      return currentOutputSpace.getCorrespondingQuality();
   }
   
   public double getPureQuality()
   {
      return currentOutputSpace.getQuality();
   }

   public int getIteration()
   {
      return iteration;
   }

   public double getComputationTime()
   {
      return computationTime;
   }

   public static Function<DMatrixRMaj, RigidBodyTransform> createSpatialInputFunction()
   {
      return new Function<DMatrixRMaj, RigidBodyTransform>()
      {
         @Override
         public RigidBodyTransform apply(DMatrixRMaj input)
         {
            RigidBodyTransform transform = new RigidBodyTransform();

            transform.setRotationYawPitchRollAndZeroTranslation(input.get(5), input.get(4), input.get(3));
            transform.getTranslation().set(input.get(0), input.get(1), input.get(2));
            return transform;
         }
      };
   }

   private class OuputSpace
   {
      private final DMatrixRMaj output;
      private final boolean[] correspondence;
      private int numberOfCorrespondingPoints;
      private double correspondingQuality;
      private double quality;

      private OuputSpace(int dimension)
      {
         output = new DMatrixRMaj(dimension, 1);
         correspondence = new boolean[dimension];
      }

      /**
       * update output.
       */
      void updateOutputSpace(DMatrixRMaj output)
      {
         this.output.set(output);
      }

      /**
       * compute correspondence and update them. return false if there is no correspondence point.
       */
      boolean computeCorrespondence()
      {
         numberOfCorrespondingPoints = 0;
         for (int i = 0; i < output.getNumRows(); i++)
         {
            if (output.get(i, 0) < correspondenceThreshold)
            {
               correspondence[i] = true;
               numberOfCorrespondingPoints++;
            }
            else
            {
               correspondence[i] = false;
            }
         }
         if (numberOfCorrespondingPoints == 0)
         {
            return false;
         }
         return true;
      }

      /**
       * there are two kinds of quality. one is distance between corresponding points
       * (correspondingQuality). the other is distance between the closest points (quality).
       */
      void computeQuality()
      {
         correspondingQuality = 0.0;
         quality = 0.0;
         for (int i = 0; i < output.getNumRows(); i++)
         {
            double norm = output.get(i, 0) * output.get(i, 0);
            quality = quality + norm;
            if (correspondence[i])
            {
               correspondingQuality = correspondingQuality + norm;
            }
         }
      }

      DMatrixRMaj getOutput()
      {
         return output;
      }

      boolean isCorresponding(int index)
      {
         return correspondence[index];
      }

      int getNumberOfCorrespondingPoints()
      {
         return numberOfCorrespondingPoints;
      }

      double getCorrespondingQuality()
      {
         return correspondingQuality;
      }

      double getQuality()
      {
         return quality;
      }
   }
}
