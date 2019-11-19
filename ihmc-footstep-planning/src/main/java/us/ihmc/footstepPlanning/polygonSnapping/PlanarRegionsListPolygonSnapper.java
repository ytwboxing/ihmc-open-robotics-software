package us.ihmc.footstepPlanning.polygonSnapping;

import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionsListPolygonSnapper
{

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2D polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo)
   {
      return snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsListToSnapTo, null);
   }

   /**
    * Snaps an XY polygon down onto a PlanarRegionsList. Returns the RigidBodyTransform required to perform the snap.
    *
    * @param polygonToSnap ConvexPolygon2d that is to be snapped to the PlanarRegionsList.
    * @param planarRegionsListToSnapTo PlanarRegionsList that the polygon will be snapped to.
    * @param regionToPack the planar region that this snaps to will be packed here (can be null).
    * @return RigidBodyTransform Transform required to snap the polygon down onto the PlanarRegion.
    */
   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, PlanarRegionsList planarRegionsListToSnapTo, PlanarRegion regionToPack)
   {
      return snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsListToSnapTo.getPlanarRegionsAsList(), regionToPack);
   }

   public static RigidBodyTransform snapPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, List<PlanarRegion> planarRegionsListToSnapTo, PlanarRegion regionToPack)
   {
      double allowableExtraZ = 0.003; // For close ones. When close, take one that is flatter...
      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(polygonToSnap, planarRegionsListToSnapTo);

      if ((intersectingRegions == null) || (intersectingRegions.isEmpty()))
      {
         return null;
      }

      int numberOfIntersectingRegions = intersectingRegions.size();

      Vector3D surfaceNormal = new Vector3D();
      Vector3D highestSurfaceNormal = new Vector3D();
      RigidBodyTransform highestTransform = null;
      double highestZ = Double.NEGATIVE_INFINITY;
      Point3D highestVertexInWorld = new Point3D();
      PlanarRegion highestPlanarRegion = null;

      for (int i = 0; i < numberOfIntersectingRegions; i++)
      {
         PlanarRegion planarRegion = intersectingRegions.get(i);

         RigidBodyTransform snapTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegion, highestVertexInWorld);

         if (highestVertexInWorld.getZ() > highestZ + allowableExtraZ)
         {
            highestZ = highestVertexInWorld.getZ();
            highestTransform = snapTransform;
            highestPlanarRegion = planarRegion;
         }

         else if (highestVertexInWorld.getZ() > highestZ - allowableExtraZ)
         {
            // Tie. Let's take the one with the flatter surface normal.
            planarRegion.getNormal(surfaceNormal);
            highestPlanarRegion.getNormal(highestSurfaceNormal);

            if (Math.abs(surfaceNormal.getZ()) > Math.abs(highestSurfaceNormal.getZ()))
            {
               highestZ = highestVertexInWorld.getZ();
               highestTransform = snapTransform;
               highestPlanarRegion = planarRegion;
            }
         }
      }

      //TODO: For now, just ignore Planar Regions that are tilted too much.
      //TODO: But later, we need to make sure they are not obstacles that need to be avoided...
      highestPlanarRegion.getNormal(highestSurfaceNormal);
      if (Math.abs(highestSurfaceNormal.getZ()) < 0.2)
         return null;

      if (regionToPack != null)
         regionToPack.set(highestPlanarRegion);
      return highestTransform;
   }

   public static RigidBodyTransform snapAndAdaptPolygonToPlanarRegionsList(ConvexPolygon2DReadOnly polygonToSnap, List<PlanarRegion> planarRegionsListToSnapTo, List<PlanarRegion> regionsToPack)
   {
      double allowableExtraZ = 0.003; // For close ones. When close, take one that is flatter...
      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(polygonToSnap, planarRegionsListToSnapTo);

      if ((intersectingRegions == null) || (intersectingRegions.isEmpty()))
      {
         return null;
      }


   }

   static Point3DReadOnly findIntersectingPointOnOtherPlaneAtADistance(Point3DReadOnly currentPoint, double yaw, PlanarRegion currentPointRegion, PlanarRegion otherRegion, double desiredDistance)
   {
      double[] planeCoefficients = getPlaneCoefficients(otherRegion.getTransformToWorld(), otherRegion.getNormal());

      double x1 = currentPoint.getX();
      double y1 = currentPoint.getY();
      double z1 = currentPoint.getZ();

      double a2 = planeCoefficients[0];
      double b2 = planeCoefficients[1];
      double c2 = planeCoefficients[2];

      double a3 = 1.0 / Math.tan(yaw);
      double b3 = -a3 * x1 + y1;

      double a = (1 + MathTools.square(a3) + MathTools.square(a2 + b2 * a3));
      double b = 2 * ((a2 + b2 * a3) * (b2 * b3 + c2 - z1) - x1 + b3 - y1);
      double c = MathTools.square(x1) + MathTools.square(b3 - y1) + MathTools.square(b2 * b3 + c2 - z1) - desiredDistance * desiredDistance;

      double x2_a = (-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a);
      double x2_b = (-b - Math.sqrt(b * b - 4 * a * c)) / (2 * a);

      double y2_a = a3 * x2_a + b3;
      double y2_b = a3 * x2_b + b3;

      double z2_a = a2 * x2_a + b2 * y2_a + c2;
      double z2_b = a2 * x2_b + b2 * y2_b + c2;

      // TODO handle the case that the point is on the current plane.
      double intersectionHeightOnCurrentPlane = currentPointRegion.getPlaneZGivenXY(x2_a, y2_a);
      if (z2_a > intersectionHeightOnCurrentPlane)
         return new Point3D(x2_a, y2_a, z2_a);
      else
         return new Point3D(x2_b, y2_b, z2_b);
   }

   static double[] getPlaneCoefficients(RigidBodyTransformReadOnly transformToWorld, Vector3DReadOnly normal)
   {
      double[] coefficients = new double[3];

      // The three components of the plane origin
      double xw = transformToWorld.getTranslation().getX();
      double yw = transformToWorld.getTranslation().getY();
      double zw = transformToWorld.getTranslation().getZ();
      // The three components of the plane normal
      double a = normal.getX();
      double b = normal.getY();
      double c = normal.getZ();

      coefficients[0] = -a / c;
      coefficients[1] = -b / c;
      coefficients[2] = a / c * xw + b / c * yw + zw;

      return coefficients;
   }
}
