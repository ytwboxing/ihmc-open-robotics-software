package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.List;

public class GeometryPolygonTools
{
   /**
    * Checks to see if the inner polygon is entirely inside the outer polygon.
    */
   public static boolean isPolygonInsideOtherPolygon(Vertex2DSupplier innerPolygon, ConcavePolygon2DReadOnly outerPolygon)
   {
      for (int i = 0; i < innerPolygon.getNumberOfVertices(); i++)
      {
         if (!outerPolygon.isPointInside(innerPolygon.getVertex(i)))
            return false;
      }

      return true;
   }

   public static boolean doPolygonsIntersect(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB)
   {
      if (!polygonA.getBoundingBox().intersectsInclusive(polygonB.getBoundingBox()))
         return false;

      return doPolygonsIntersectBruteForce(polygonA, polygonB);
   }

   public static boolean doPolygonsIntersectBruteForce(ConcavePolygon2DReadOnly polygonA, ConcavePolygon2DReadOnly polygonB)
   {
      for (int i = 0; i < polygonA.getNumberOfVertices(); i++)
      {
         Point2DReadOnly startVertex = polygonA.getVertex(i);
         Point2DReadOnly endVertex = polygonA.getVertex(EuclidGeometryPolygonTools.next(i, polygonA.getNumberOfVertices()));

         if (doesLineSegment2DIntersectPolygon(startVertex, endVertex, polygonB))
            return true;
      }

      return false;
   }

   public static boolean doesLineSegment2DIntersectPolygon(Point2DReadOnly segmentStart, Point2DReadOnly segmentEnd, ConcavePolygon2DReadOnly polygon)
   {
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly startVertex = polygon.getVertex(i);
         Point2DReadOnly endVertex = polygon.getVertex(EuclidGeometryPolygonTools.next(i, polygon.getNumberOfVertices()));

         if (EuclidGeometryTools.doLineSegment2DsIntersect(segmentStart, segmentStart, startVertex, endVertex))
            return true;
      }

      return false;
   }


   public static boolean isClockwiseOrdered(List<? extends Point2DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      checkNumberOfVertices(concaveHullVertices, numberOfVertices);

      double sumOfAngles = 0.0;

      for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
      {
         int previousVertexIndex = EuclidGeometryPolygonTools.previous(vertexIndex, numberOfVertices);
         int nextVertexIndex = EuclidGeometryPolygonTools.next(vertexIndex, numberOfVertices);

         Point2DReadOnly previousVertex = concaveHullVertices.get(previousVertexIndex);
         Point2DReadOnly vertex = concaveHullVertices.get(vertexIndex);
         Point2DReadOnly nextVertex = concaveHullVertices.get(nextVertexIndex);

         double firstVectorX = vertex.getX() - previousVertex.getX();
         double firstVectorY = vertex.getY() - previousVertex.getY();
         double secondVectorX = nextVertex.getX() - vertex.getX();
         double secondVectorY = nextVertex.getY() - vertex.getY();
         sumOfAngles += angle(firstVectorX, firstVectorY, secondVectorX, secondVectorY);
      }

      return sumOfAngles <= 0.0;
   }

   /**
    * Solver to determine if a point is inside or outside a simple polygon. A simple polygon is a convex or concave polygon with no self intersections and no
    * holes.
    * The solver performs a ray-cast from the query point along the x-axis. If a ray has an odd number of intersections, that indicates the point is inside the
    * polygon.
    *
    * @see <a href="https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm</a>
    * @see <a href="https://dl.acm.org/doi/10.1145/368637.368653</a>
    */
   public static boolean isPoint2DInsideSimplePolygon2D(Point2DReadOnly queryPoint, List<? extends Point2DReadOnly> polygon, int numberOfVertices)
   {
      return isPoint2DInsideSimplePolygon2D(queryPoint.getX(), queryPoint.getY(), polygon, numberOfVertices);
   }

   /**
    * Solver to determine if a point is inside or outside a simple polygon. A simple polygon is a convex or concave polygon with no self intersections and no
    * holes.
    * The solver performs a ray-cast from the query point along the x-axis. If a ray has an odd number of intersections, that indicates the point is inside the
    * polygon.
    *
    * @see <a href="https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm</a>
    * @see <a href="https://dl.acm.org/doi/10.1145/368637.368653</a>
    */
   public static boolean isPoint2DInsideSimplePolygon2D(double pointX, double pointY, List<? extends Point2DReadOnly> polygon, int numberOfVertices)
   {
      return isPoint2DInsideSimplePolygon2D(pointX, pointY, 1.0, 0.0, polygon, numberOfVertices, null);
   }

   public static boolean isPoint2DInsideSimplePolygon2D(double pointX,
                                                        double pointY,
                                                        double lineDirectionX,
                                                        double lineDirectionY,
                                                        List<? extends Point2DReadOnly> polygon,
                                                        int numberOfVertices,
                                                        Point2DBasics intersectionToPack)
   {
      checkNumberOfVertices(polygon, numberOfVertices);

      if (numberOfVertices < 3)
      {
         return false;
      }

      int intersections = 0;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex = polygon.get(i);
         Point2DReadOnly nextVertex = polygon.get((i + 1) % numberOfVertices);

         boolean intersects = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(pointX,
                                                                                           pointY,
                                                                                           lineDirectionX,
                                                                                           lineDirectionY,
                                                                                           vertex.getX(),
                                                                                           vertex.getY(),
                                                                                           nextVertex.getX(),
                                                                                           nextVertex.getY(),
                                                                                           intersectionToPack);

         if (intersects)
         {
            intersections++;
         }
      }

      boolean oddNumberOfIntersections = intersections % 2 == 1;
      return oddNumberOfIntersections;
   }

   public static boolean isSimplePolygon(List<? extends Point2DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      // TODO implement some other versions of this algorithm that are faster (see https://www.webcitation.org/6ahkPQIsN)
      return isSimplePolygonBruteForce(concaveHullVertices, numberOfVertices);
   }

   public static boolean isSimplePolygonBruteForce(List<? extends Point2DReadOnly> concaveHullVertices, int numberOfVertices)
   {
      for (int index = 0; index < numberOfVertices; index++)
      {
         int previousIndex = EuclidGeometryPolygonTools.previous(index, numberOfVertices);

         Point2DReadOnly segmentStart = concaveHullVertices.get(previousIndex);
         Point2DReadOnly segmentEnd = concaveHullVertices.get(index);

         int nextSegmentStart = EuclidGeometryPolygonTools.next(index, numberOfVertices);
         int nextSegmentEnd = EuclidGeometryPolygonTools.next(nextSegmentStart, numberOfVertices);
         while (nextSegmentEnd != previousIndex)
         {
            if (EuclidGeometryTools.doLineSegment2DsIntersect(segmentStart,
                                                              segmentEnd,
                                                              concaveHullVertices.get(nextSegmentStart),
                                                              concaveHullVertices.get(nextSegmentEnd)))
            {
               return false;
            }

            nextSegmentStart = nextSegmentEnd;
            nextSegmentEnd = EuclidGeometryPolygonTools.next(nextSegmentStart, numberOfVertices);
         }
      }

      return true;
   }

   private static double angle(Vector2DReadOnly firstVector, Vector2DReadOnly secondVector)
   {
      return angle(firstVector.getX(), firstVector.getY(), secondVector.getX(), secondVector.getY());
   }

   private static double angle(double firstVectorX, double firstVectorY, double secondVectorX, double secondVectorY)
   {
      // The sign of the angle comes from the cross product
      double crossProduct = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      // the magnitude of the angle comes from the dot product
      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY;

      return EuclidCoreTools.atan2(crossProduct, dotProduct);
   }

   private static void checkNumberOfVertices(List<? extends Point2DReadOnly> convexPolygon2D, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > convexPolygon2D.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + convexPolygon2D.size() + "].");
   }
}
