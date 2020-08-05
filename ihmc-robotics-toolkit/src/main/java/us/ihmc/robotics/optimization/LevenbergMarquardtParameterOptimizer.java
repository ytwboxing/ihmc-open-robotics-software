package us.ihmc.robotics.optimization;

import java.util.function.Function;
import java.util.function.UnaryOperator;

import gnu.trove.iterator.TIntIterator;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.matrixlib.MatrixTools;

/**
 * this class is to iterate minimizing input parameters for the given output calculator matrix. in
 * general, the input parameters are related six-dimensional in spatial space to transform rigid
 * body. for example, we may let the rigid body with a camera frame and minimize the distance
 * between two frames.
 * <p>
 * the way to use this is simple like defining input function {@link Function<DMatrixRMaj,
 * RigidBodyTransform>} and output calculator {@link UnaryOperator<DMatrixRMaj>}. and do
 * {@code initialize()} and {@code iterate()}
 * <p>
 * the method {@code iterate()} updates the input space with the following equation.
 * <p>
 * {@linkplain -inv(J^T * J + W_N) * J^T * e }
 * <p>
 * see the following examples and slides.
 * <p>
 * {@link LevenbergMarquardtICPTest}, {@link LevenbergMarquardtICPVisualizer}.
 * <p>
 * @see <a href="https://docs.google.com/presentation/d/1GH2QqMLM1cKgll5hcd-nX18c-cGVjB5ABkz_voOZU1w/edit?usp=sharing">20200722_LearningLunch_Inho</a>
 */
public class LevenbergMarquardtParameterOptimizer
{
   private static final boolean DEBUG = false;
   private int inputDimension;

   private final Function<DMatrixRMaj, RigidBodyTransform> inputFunction;
   private final OutputCalculator outputCalculator;

   private final DMatrixRMaj currentInput;
   private final OutputSpace currentOutputSpace;
   private final DMatrixRMaj perturbationVector;
   private final DMatrixRMaj perturbedInput;
   private final DMatrixRMaj jacobian;

   private final DMatrixRMaj squaredJacobian;
   private final DMatrixRMaj dampingMatrix;
   private final DMatrixRMaj invMultJacobianTranspose;

   private final DMatrixRMaj optimizeDirection;

   /**
    * higher : make quick approach when the model and the data are in long distance.
    * <p>
    * lower : slower approach but better accuracy in near distance.
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
   public LevenbergMarquardtParameterOptimizer(Function<DMatrixRMaj, RigidBodyTransform> inputFunction,
                                               OutputCalculator outputCalculator,
                                               int inputParameterDimension,
                                               int outputDimension)
   {
      this.inputFunction = inputFunction;
      this.inputDimension = inputParameterDimension;
      this.outputCalculator = outputCalculator;

      currentInput = new DMatrixRMaj(inputParameterDimension, 1);
      currentOutputSpace = new OutputSpace(outputDimension);

      perturbationVector = new DMatrixRMaj(inputParameterDimension, 1);
      CommonOps_DDRM.fill(perturbationVector, DEFAULT_PERTURBATION);
      perturbedInput = new DMatrixRMaj(inputParameterDimension, 1);
      jacobian = new DMatrixRMaj(outputDimension, inputParameterDimension);

      squaredJacobian = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      dampingMatrix = new DMatrixRMaj(inputParameterDimension, inputParameterDimension);
      invMultJacobianTranspose = new DMatrixRMaj(inputParameterDimension, outputDimension);

      optimizeDirection = new DMatrixRMaj(inputParameterDimension, 1);
   }

   public void setPerturbationVector(DMatrixRMaj perturbationVector)
   {
      if (this.perturbationVector.getNumCols() != perturbationVector.getNumCols())
         throw new MatrixDimensionException("dimension is wrong. " + this.perturbationVector.getNumCols() + " " + perturbationVector.getNumCols());
      this.perturbationVector.set(perturbationVector);
   }

   public void setCorrespondenceThreshold(double correspondenceThreshold)
   {
      this.correspondenceThreshold = correspondenceThreshold;
   }

   public boolean initialize()
   {
      iteration = 0;
      optimized = false;
      MatrixTools.setDiagonal(dampingMatrix, DEFAULT_DAMPING_COEFFICIENT);

      outputCalculator.resetIndicesToCompute();
      currentOutputSpace.updateOutputSpace(outputCalculator.apply(currentInput));

      boolean result = currentOutputSpace.computeCorrespondence();
//      currentOutputSpace.computeQuality();
      
      return result;
   }

   public double iterate()
   {
      iteration++;
      long startTime = System.nanoTime();

      // TODO because of the way the operations are ordered, on the first iteration, the current output space is already valid, as it was called in the
      // TODO initialize function. This is a relatively expensive operation, so figuring out how to reorder things would reduce the number of "update output
      // TODO space" calls by 1, which would be helpful.
      outputCalculator.resetIndicesToCompute();
      currentOutputSpace.updateOutputSpace(outputCalculator.apply(currentInput));

      if (!currentOutputSpace.computeCorrespondence())
      {
         return -1;
      }
      currentOutputSpace.computeQuality();
      outputCalculator.setIndicesToCompute(currentOutputSpace.correspondingIndices);

      int numberOfCorrespondences = currentOutputSpace.getNumberOfCorrespondingPoints();
      jacobian.reshape(numberOfCorrespondences, inputDimension);
      invMultJacobianTranspose.reshape(inputDimension, numberOfCorrespondences);

      // start.      
      // compute jacobian.
      perturbedInput.set(currentInput);
      for (int i = 0; i < inputDimension; i++)
      {
         perturbedInput.add(i, 0, perturbationVector.get(i));

         DMatrixRMaj perturbedOutput = outputCalculator.apply(perturbedInput);
         if (perturbedOutput.getNumRows() != numberOfCorrespondences)
            throw new IllegalArgumentException("Got a size problem.");

         for (int j = 0; j < numberOfCorrespondences; j++)
         {
               double partialValue = (perturbedOutput.get(j) - currentOutputSpace.getCorrespondingOutput().get(j)) / perturbationVector.get(i);
               jacobian.set(j, i, partialValue);
         }

         perturbedInput.add(i, 0, -perturbationVector.get(i));
      }

      // compute direction.
      CommonOps_DDRM.multInner(jacobian, squaredJacobian);
      if (useDamping)
      {
         CommonOps_DDRM.addEquals(squaredJacobian, dampingMatrix);
      }
      CommonOps_DDRM.invert(squaredJacobian);

      CommonOps_DDRM.multTransB(squaredJacobian, jacobian, invMultJacobianTranspose);
      CommonOps_DDRM.mult(invMultJacobianTranspose, currentOutputSpace.getCorrespondingOutput(), optimizeDirection);

      // update currentInput.
      CommonOps_DDRM.subtract(currentInput, optimizeDirection, currentInput);

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

   public int getNumberOfCorrespondingPoints()
   {
      return currentOutputSpace.getNumberOfCorrespondingPoints();
   }

   public DMatrixRMaj getOptimalParameter()
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

   public static Function<DMatrixRMaj, RigidBodyTransform> createSpatialInputFunction(boolean includePitchAndRoll)
   {
      return new Function<DMatrixRMaj, RigidBodyTransform>()
      {
         @Override
         public RigidBodyTransform apply(DMatrixRMaj input)
         {
            RigidBodyTransform transform = new RigidBodyTransform();

            if (includePitchAndRoll)
               transform.setRotationYawPitchRollAndZeroTranslation(input.get(5), input.get(4), input.get(3));
            else
               transform.setRotationYawAndZeroTranslation(input.get(3));
            transform.getTranslation().set(input.get(0), input.get(1), input.get(2));
            return transform;
         }
      };
   }

   private class OutputSpace
   {
      private final DMatrixRMaj output;
      private DMatrixRMaj correspondingOutput;
      private final boolean[] correspondence;
      private final TIntArrayList correspondingIndices = new TIntArrayList();
      private double correspondingQuality;
      private double quality;

      private OutputSpace(int dimension)
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
         correspondingIndices.clear();
         for (int i = 0; i < output.getNumRows(); i++)
         {
            if (output.get(i, 0) < correspondenceThreshold)
            {
               correspondence[i] = true;
               correspondingIndices.add(i);
            }
            else
            {
               correspondence[i] = false;
            }
         }

         correspondingOutput = new DMatrixRMaj(correspondingIndices.size(), 1);
         int index = 0;
         TIntIterator iterator = correspondingIndices.iterator();
         while (iterator.hasNext())
            correspondingOutput.set(index++, 0, output.get(iterator.next()));

         return correspondingIndices.size() != 0;
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
            quality += norm;
            if (correspondence[i])
            {
               correspondingQuality += norm;
            }
         }
      }

      DMatrixRMaj getOutput()
      {
         return output;
      }

      DMatrixRMaj getCorrespondingOutput()
      {
         return correspondingOutput;
      }

      int getNumberOfCorrespondingPoints()
      {
         return correspondingIndices.size();
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
