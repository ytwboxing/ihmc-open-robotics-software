package us.ihmc.robotics.geometry.concaveHull.weilerAtherton;

import org.junit.jupiter.api.Test;
import us.ihmc.robotics.geometry.concaveHull.GeometryPolygonTestTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.PolygonClippingAndMerging;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.*;

public class PolygonClippingAndMergingTest
{
   @Test
   public void testClippingRemoveSquareChunkFromSideOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(1.0 + -0.5, 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, -0.5);
      clippingPolygon.addVertex(1.0 + -0.5, -0.5);
      clippingPolygon.update();

      assertTrue(polygonToClip.isPointInside(clippingPolygon.getVertex(0)));
      assertFalse(polygonToClip.isPointInside(clippingPolygon.getVertex(1)));
      assertFalse(polygonToClip.isPointInside(clippingPolygon.getVertex(2)));
      assertTrue(polygonToClip.isPointInside(clippingPolygon.getVertex(3)));

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(0.5, -0.5);
      clippedPolygonExpected.addVertex(1.0, -0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testClippingThatBreaksPolygonInHalf()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();


      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.05, 2.0);
      clippingPolygon.addVertex(0.05, -2.0);
      clippingPolygon.addVertex(-0.05, -2.0);
      clippingPolygon.addVertex(-0.05, 2.0);
      clippingPolygon.update();

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected1 = new ConcavePolygon2D();
      clippedPolygonExpected1.addVertex(-1.0, 1.0);
      clippedPolygonExpected1.addVertex(-0.05, 1.0);
      clippedPolygonExpected1.addVertex(-0.05, -1.0);
      clippedPolygonExpected1.addVertex(-1.0, -1.0);
      clippedPolygonExpected1.update();

      ConcavePolygon2D clippedPolygonExpected2 = new ConcavePolygon2D();
      clippedPolygonExpected2.addVertex(0.05, 1.0);
      clippedPolygonExpected2.addVertex(1.0, 1.0);
      clippedPolygonExpected2.addVertex(1.0, -1.0);
      clippedPolygonExpected2.addVertex(0.05, -1.0);
      clippedPolygonExpected2.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(2, clippedPolygons.size());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected2, clippedPolygons.get(0), 1e-7);
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected1, clippedPolygons.get(1), 1e-7);
   }

   @Test
   public void testMergeAddingSquareChunkFromSideOfSquare()
   {
      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.0);
      polygonB.addVertex(1.0, 1.0);
      polygonB.addVertex(1.0, -1.0);
      polygonB.addVertex(-1.0, -1.0);
      polygonB.update();

      assertTrue(polygonB.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(1.0 + -0.5, 0.5);
      polygonA.addVertex(1.0 + 0.5, 0.5);
      polygonA.addVertex(1.0 + 0.5, -0.5);
      polygonA.addVertex(1.0 + -0.5, -0.5);
      polygonA.update();


      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, 0.5);
      mergedPolygonExpected.addVertex(1.5, 0.5);
      mergedPolygonExpected.addVertex(1.5, -0.5);
      mergedPolygonExpected.addVertex(1.0, -0.5);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testDoPolygonsIntersectWithNoCrossContainedPoints()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(1.0, 0.5);
      polygonA.addVertex(1.0, -0.5);
      polygonA.addVertex(-1.0, -0.5);
      polygonA.addVertex(-1.0, 0.5);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.5, 1.0);
      polygonB.addVertex(0.5, -1.0);
      polygonB.addVertex(-0.5, -1.0);
      polygonB.addVertex(-0.5, 1.0);
      polygonB.update();

      assertFalse(polygonA.getVertexBufferView().stream().anyMatch(polygonB::isPointInside));
      assertFalse(polygonB.getVertexBufferView().stream().anyMatch(polygonA::isPointInside));
      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonA, polygonB));
   }


   @Test
   public void testClipRemoveSquareChunkFromCornerOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(1.0 + -0.5, 1.0 + 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 1.0 + 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 1.0 - 0.5);
      clippingPolygon.addVertex(1.0 + -0.5, 1.0 - 0.5);
      clippingPolygon.update();

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergeAllPossible()
   {
      ConcavePolygon2D polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      ConcavePolygon2D polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(0.9, 1.0);
      polygon2.addVertex(1.1, 1.0);
      polygon2.addVertex(1.1, -1.0);
      polygon2.addVertex(0.9, -1.0);
      polygon2.update();

      ConcavePolygon2D polygon3 = new ConcavePolygon2D();
      polygon3.addVertex(-0.09, 0.1);
      polygon3.addVertex(1.0, 0.1);
      polygon3.addVertex(1.0, -0.1);
      polygon3.addVertex(-0.09, -0.1);
      polygon3.update();

      List<ConcavePolygon2DBasics> polygons = new ArrayList<>();
      polygons.add(polygon1);
      polygons.add(polygon2);
      polygons.add(polygon3);

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      mergedPolygon.addVertex(-0.1, 1.0);
      mergedPolygon.addVertex(0.1, 1.0);
      mergedPolygon.addVertex(0.1, 0.1);
      mergedPolygon.addVertex(0.9, 0.1);
      mergedPolygon.addVertex(0.9, 1.0);
      mergedPolygon.addVertex(1.1, 1.0);
      mergedPolygon.addVertex(1.1, -1.0);
      mergedPolygon.addVertex(0.9, -1.0);
      mergedPolygon.addVertex(0.9, -0.1);
      mergedPolygon.addVertex(0.1, -0.1);
      mergedPolygon.addVertex(0.1, -1.0);
      mergedPolygon.addVertex(-0.1, -1.0);
      mergedPolygon.update();

      PolygonClippingAndMerging.mergeAllPossible(polygons);

      assertEquals(1, polygons.size());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygon, polygons.get(0), 1e-7);

      polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(0.9, 0.1);
      polygon2.addVertex(1.0, 0.1);
      polygon2.addVertex(1.0, -0.1);
      polygon2.addVertex(0.9, -0.1);
      polygon2.update();

      polygon3 = new ConcavePolygon2D();
      polygon3.addVertex(-0.1, 0.1);
      polygon3.addVertex(1.0, 0.1);
      polygon3.addVertex(1.0, -0.1);
      polygon3.addVertex(-0.1, -0.1);
      polygon3.update();

      polygons = new ArrayList<>();
      polygons.add(polygon1);
      polygons.add(polygon2);
      polygons.add(polygon3);

      PolygonClippingAndMerging.mergeAllPossible(polygons);

      assertEquals(1, polygons.size());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygon, polygons.get(0), 1e-7);

      fail("Need to check the actual merged polygons.");
   }




   @Test
   public void testMergeAddSquareChunkFromCornerOfSquare()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      assertTrue(polygonA.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(1.0 + -0.5, 1.0 + 0.5);
      polygonB.addVertex(1.0 + 0.5, 1.0 + 0.5);
      polygonB.addVertex(1.0 + 0.5, 1.0 - 0.5);
      polygonB.addVertex(1.0 + -0.5, 1.0 - 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 0.5);
      mergedPolygonExpected.addVertex(1.0, 0.5);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClippingWithTriangleClippingCornerOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.0, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(1.5, 0.0);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergingWithTriangleClippingCornerOfSquare()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      assertTrue(polygonA.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.0, 1.5);
      polygonB.addVertex(1.5, 1.5);
      polygonB.addVertex(1.5, 0.0);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(0.0, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 0.0);
      mergedPolygonExpected.addVertex(1.0, 0.5);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClippingWithTriangleOnTopEdgeOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.0, 1.5);
      clippingPolygon.addVertex(1.0, 1.5);
      clippingPolygon.addVertex(0.0, 0.5);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(-0.5, 1.0);
      clippedPolygonExpected.addVertex(0.0, 0.5);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);

      clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.5, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(0.0, 0.0);
      clippingPolygon.update();

      clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.0, 0.0);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergingWithTriangleOnTopEdgeOfSquare()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();


      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.5);
      polygonB.addVertex(1.0, 1.5);
      polygonB.addVertex(0.0, 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(-0.5, 1.0);
      mergedPolygonExpected.addVertex(-1.0, 1.5);
      mergedPolygonExpected.addVertex(1.0, 1.5);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));

      polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.5, 1.5);
      polygonB.addVertex(1.5, 1.5);
      polygonB.addVertex(0.0, 0.0);
      polygonB.update();

      mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(-1.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClippingRemoveComplexShapeAcrossTopEdge()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.5, 1.0);
      polygonToClip.addVertex(2.0, 1.0);
      polygonToClip.addVertex(2.0, -1.0);
      polygonToClip.addVertex(-1.5, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.0, 2.0);
      clippingPolygon.addVertex(2.0, 2.0);
      clippingPolygon.addVertex(2.0, 1.5);
      clippingPolygon.addVertex(1.0, 0.5);
      clippingPolygon.addVertex(0.0, 1.5);
      clippingPolygon.addVertex(-1.0, 0.5);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.5, 1.0);
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(-1.0, 0.5);
      clippedPolygonExpected.addVertex(-0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(1.5, 1.0);
      clippedPolygonExpected.addVertex(2.0, 1.0);
      clippedPolygonExpected.addVertex(2.0, -1.0);
      clippedPolygonExpected.addVertex(-1.5, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergingComplexShapeAcrossTopEdge()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.5, 1.0);
      polygonA.addVertex(2.0, 1.0);
      polygonA.addVertex(2.0, -1.0);
      polygonA.addVertex(-1.5, -1.0);
      polygonA.update();

      assertTrue(polygonA.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 2.0);
      polygonB.addVertex(2.0, 2.0);
      polygonB.addVertex(2.0, 1.5);
      polygonB.addVertex(1.0, 0.5);
      polygonB.addVertex(0.0, 1.5);
      polygonB.addVertex(-1.0, 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.5, 1.0);
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(-1.0, 2.0);
      mergedPolygonExpected.addVertex(2.0, 2.0);
      mergedPolygonExpected.addVertex(2.0, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.0);
      mergedPolygonExpected.addVertex(2.0, 1.0);
      mergedPolygonExpected.addVertex(2.0, -1.0);
      mergedPolygonExpected.addVertex(-1.5, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClipWithTrickyOverlappingJoints()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.5, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(0.5, 0.5);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergeWithTrickyOverlappingJoints()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.5, 1.5);
      polygonB.addVertex(1.5, 1.5);
      polygonB.addVertex(0.5, 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

}