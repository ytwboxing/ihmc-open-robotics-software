package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SegmentedImageRawData;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

import java.util.stream.Stream;

public class SuperPixelNormalEstimationTools
{
   private static final boolean addPointsInParallel = true;

   public static void updateUsingPCA(SegmentedImageRawData superPixel)
   {
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
      Stream<Point3D> pointStream = addPointsInParallel ? superPixel.getPoints().parallelStream() : superPixel.getPoints().stream();
      pointStream.forEach(pca::addDataPoint);

      pca.compute();

      Point3D center = new Point3D();
      Vector3D normal = new Vector3D();
      Vector3D standardDeviation = new Vector3D();

      if (normal.getZ() < 0.0)
         normal.negate();

      pca.getMean(center);
      pca.getThirdVector(normal);
      pca.getStandardDeviation(standardDeviation);

      superPixel.setCenter(center);
      superPixel.setNormal(normal);
      superPixel.setStandardDeviation(standardDeviation);
   }
}
