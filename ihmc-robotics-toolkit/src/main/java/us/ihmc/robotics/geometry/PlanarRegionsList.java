package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class PlanarRegionsList
{
   private final List<PlanarRegion> regions;

   public PlanarRegionsList(PlanarRegion... planarRegions)
   {
      regions = new ArrayList<>();
      for (PlanarRegion planarRegion : planarRegions)
      {
         regions.add(planarRegion);
      }
   }

   public PlanarRegionsList(List<PlanarRegion> planarRegions)
   {
      regions = planarRegions;
   }

   public PlanarRegionsList(PlanarRegionsList other)
   {
      this(other.getPlanarRegionsAsList());
   }

   /**
    * Adds a planar region to this list of planar regions.
    *
    * @param region to add.
    */
   public void addPlanarRegion(PlanarRegion region)
   {
      regions.add(region);
   }

   public void addPlanarRegions(List<PlanarRegion> regions)
   {
      this.regions.addAll(regions);
   }

   public void addPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      regions.addAll(planarRegionsList.getPlanarRegionsAsList());
   }

   /**
    * Clears the planar regions list.
    */
   public void clear()
   {
      regions.clear();
   }


   /**
    * Find all the planar regions that intersect with the given 2d line segment. The algorithm is
    * equivalent to projecting all the regions onto the XY-plane and then finding the regions
    * intersecting with the given line segment.
    *
    * @param lineSegmentInWorld the query.
    * @param intersectingRegionsToPack ArrayList were the intersecting regions will be packed into.
    */
   public void findPlanarRegionsIntersectingLineSegment(LineSegment2DReadOnly lineSegmentInWorld, List<PlanarRegion> intersectingRegionsToPack)
   {
      PlanarRegionTools.findPlanarRegionsIntersectingLineSegment(lineSegmentInWorld, intersectingRegionsToPack, regions);
   }

   /**
    * Find all the planar regions that intersect with the given 2d line segment. The algorithm is
    * equivalent to projecting all the regions onto the XY-plane and then finding the regions
    * intersecting with the given line segment.
    *
    * @param pointInWorld the query.
    * @param intersectingRegionsToPack ArrayList were the intersecting regions will be packed into.
    */
   public void findPlanarRegionsWithinEpsilonOfPoint(Point3DReadOnly pointInWorld, double epsilon, List<PlanarRegion> intersectingRegionsToPack)
   {
      PlanarRegionTools.findPlanarRegionsWithinEpsilonOfPoint(pointInWorld, regions, epsilon, intersectingRegionsToPack);
   }

   /**
    * Find all the planar regions that contain the given point.
    *
    * @param point the query coordinates.
    * @param maximumOrthogonalDistance tolerance expressed as maximum orthogonal distance from the
    *           region.
    * @return the list of planar regions containing the query. Returns null when no region contains
    *         the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPoint(Point3DReadOnly point, double maximumOrthogonalDistance)
   {
      return PlanarRegionTools.findPlanarRegionsContainingPoint(point, regions, maximumOrthogonalDistance);
   }

   /**
    * Find all the planar regions that contain the given point. The algorithm is equivalent to
    * projecting all the regions onto the XY-plane and then finding the regions containing the
    * point.
    *
    * @param point the query coordinates.
    * @return the list of planar regions containing the query. Returns null when no region contains
    *         the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPointByProjectionOntoXYPlane(Point2DReadOnly point)
   {
      return PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(point, regions);
   }

   /**
    * Find all the planar regions that contain the given point. The algorithm is equivalent to
    * projecting all the regions onto the XY-plane and then finding the regions containing the
    * point.
    *
    * @param x the query x-coordinate.
    * @param y the query y-coordinate.
    * @return the list of planar regions containing the query. Returns null when no region contains
    *         the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPointByProjectionOntoXYPlane(double x, double y)
   {
      return PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(x, y, regions);
   }

   public List<PlanarRegion> findPlanarRegionsContainingPointByVerticalLineIntersection(Point2DReadOnly point)
   {
      return PlanarRegionTools.findPlanarRegionsContainingPointByVerticalLineIntersection(point, regions);
   }

   /**
    * Find the closest planar region to the given point. The algorithm is equivalent to
    * projecting all the regions onto the XY-plane and then finding the closest one to the point.
    *
    * @param point the query coordinates.
    * @return the planar regions closest to the query.
    */
   public PlanarRegion findClosestPlanarRegionToPointByProjectionOntoXYPlane(Point2DReadOnly point)
   {
      return PlanarRegionTools.findClosestPlanarRegionToPointByProjectionOntoXYPlane(point, regions);
   }

   /** Returns true if this list of planar regions is empty (contains no planar regions). */
   public boolean isEmpty()
   {
      return regions.isEmpty();
   }

   /** Return the number of planar regions contained in this list. */
   public int getNumberOfPlanarRegions()
   {
      return regions.size();
   }

   /** Retrieves the i<sup>th</sup> planar region of this list. */
   public PlanarRegion getPlanarRegion(int index)
   {
      return regions.get(index);
   }

   /** Retrieves the planar regions as a {@code List}. */
   public List<PlanarRegion> getPlanarRegionsAsList()
   {
      return regions;
   }

   /**
    * Retrieves the last planar region of this list. Special case: returns null when the list is
    * empty.
    */
   public PlanarRegion getLastPlanarRegion()
   {
      if (isEmpty())
         return null;
      else
         return getPlanarRegion(getNumberOfPlanarRegions() - 1);
   }

   /** Retrieves the i<sup>th</sup> planar region of this list and removes it from this list. */
   public PlanarRegion pollPlanarRegion(int index)
   {
      return regions.remove(index);
   }

   /**
    * Retrieves the last planar region of this list and removes it from this list. Special case:
    * returns null when the list is empty.
    */
   public PlanarRegion pollLastPlanarRegion()
   {
      if (isEmpty())
         return null;
      else
         return pollPlanarRegion(getNumberOfPlanarRegions() - 1);
   }

   /**
    * @return a full depth copy of this list of planar regions. The copy can be entirely modified
    *         without interfering with this.
    */
   public PlanarRegionsList copy()
   {
      List<PlanarRegion> planarRegionsCopy = new ArrayList<>();

      for (int i = 0; i < getNumberOfPlanarRegions(); i++)
         planarRegionsCopy.add(regions.get(i).copy());

      return new PlanarRegionsList(planarRegionsCopy);
   }

   /**
    * Transforms the planar regions list
    *
    * @param rigidBodyTransform transform from current frame to desired frame
    */
   public void transform(RigidBodyTransform rigidBodyTransform)
   {
      for (int i = 0; i < regions.size(); i++)
      {
         regions.get(i).transform(rigidBodyTransform);
      }
   }

   public void transformByPreMultiply(RigidBodyTransform rigidBodyTransform)
   {
      for (int i = 0; i < regions.size(); i++)
      {
         regions.get(i).transformByPreMultiply(rigidBodyTransform);
      }
   }

   public static PlanarRegionsList generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(Random random, int numberOfRandomlyGeneratedPolygons,
                                                                                                  double maxAbsoluteXYForPolygons,
                                                                                                  int numberOfPossiblePointsForPolygons,
                                                                                                  int numberOfPossiblePlanarRegions)
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      int numberOfPlanarRegions = random.nextInt(numberOfPossiblePlanarRegions) + 1;
      while (planarRegionsList.getNumberOfPlanarRegions() < numberOfPlanarRegions)
      {
         planarRegionsList.addPlanarRegion(PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, numberOfRandomlyGeneratedPolygons,
                                                                                                                  maxAbsoluteXYForPolygons,
                                                                                                                  numberOfPossiblePointsForPolygons));
      }
      return planarRegionsList;
   }

   public static PlanarRegionsList flatGround(double size)
   {
      return flatGround(size, new RigidBodyTransform());
   }

   public static PlanarRegionsList flatGround(double size, RigidBodyTransform transform)
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();  // start with a flat ground region
      double halfSize = size / 2.0;
      convexPolygon.addVertex(halfSize, halfSize);
      convexPolygon.addVertex(-halfSize, halfSize);
      convexPolygon.addVertex(-halfSize, -halfSize);
      convexPolygon.addVertex(halfSize, -halfSize);
      convexPolygon.update();
      PlanarRegion groundPlane = new PlanarRegion(transform, convexPolygon);
      return new PlanarRegionsList(groundPlane);
   }
}