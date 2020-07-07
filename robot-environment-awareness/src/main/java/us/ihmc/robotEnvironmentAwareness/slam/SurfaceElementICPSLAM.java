package us.ihmc.robotEnvironmentAwareness.slam;

<<<<<<< HEAD
import org.ejml.data.DenseMatrix64F;
=======
import java.util.function.UnaryOperator;

import org.ejml.data.DMatrixRMaj;
>>>>>>> 2d0a07337e7... Replaced function output with UnaryOperator.

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;

public class SurfaceElementICPSLAM extends SLAMBasics
{
   public static final boolean DEBUG = false;

   public SurfaceElementICPSLAM(double octreeResolution)
   {
      super(octreeResolution);
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.05;
      int minimumNumberOfHits = 1;
      frame.registerSurfaceElements(octree, windowMargin, surfaceElementResolution, minimumNumberOfHits, false);

      int numberOfSurfel = frame.getSurfaceElementsToSensor().size();
      UnaryOperator<DMatrixRMaj> outputCalculator = new UnaryOperator<DMatrixRMaj>()
      {
         @Override
<<<<<<< HEAD
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
=======
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
>>>>>>> 2d0a07337e7... Replaced function output with UnaryOperator.
         {
            RigidBodyTransform driftCorrectionTransform = convertTransform(inputParameter.getData());
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame.getOriginalSensorPose());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame.getSurfaceElementsToSensor().get(i));

               correctedSensorPoseToWorld.transform(correctedSurfel[i].getPoint());
               correctedSensorPoseToWorld.transform(correctedSurfel[i].getNormal());
            }

            DenseMatrix64F errorSpace = new DenseMatrix64F(correctedSurfel.length, 1);
            for (int i = 0; i < correctedSurfel.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfel[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeBoundedPerpendicularDistancePointToNormalOctree(octree, surfel.getPoint(), octree.getResolution());
         }
      };
<<<<<<< HEAD
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
=======
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, numberOfSurfel, outputCalculator);
      DMatrixRMaj purterbationVector = new DMatrixRMaj(6, 1);
>>>>>>> 2d0a07337e7... Replaced function output with UnaryOperator.
      purterbationVector.set(0, 0.0005);
      purterbationVector.set(1, 0.0005);
      purterbationVector.set(2, 0.0005);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      // do ICP.
      for (int i = 0; i < 30; i++)
      {
         optimizer.iterate();
      }

      // get parameter.
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      icpTransformer.set(convertTransform(optimizer.getOptimalParameter().getData()));

      return icpTransformer;
   }

   private RigidBodyTransform convertTransform(double... transformParameters)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(transformParameters[0], transformParameters[1], transformParameters[2]);
      transform.appendRollRotation(transformParameters[3]);
      transform.appendPitchRotation(transformParameters[4]);
      transform.appendYawRotation(transformParameters[5]);

      return transform;
   }
}
