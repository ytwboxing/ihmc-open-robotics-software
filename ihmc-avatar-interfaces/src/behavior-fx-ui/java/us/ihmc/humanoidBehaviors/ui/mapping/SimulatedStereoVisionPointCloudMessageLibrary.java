package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class SimulatedStereoVisionPointCloudMessageLibrary
{
   private static final int NUMBER_OF_POINTS = 20000;

   public static StereoVisionPointCloudMessage generateMessageSimpleStair(RigidBodyTransformReadOnly sensorPose,
                                                                          RigidBodyTransformReadOnly stairOriginTransform, double lowerLength,
                                                                          double upperLength, double width, double height,
                                                                          RigidBodyTransformReadOnly perturbedTransformer, boolean generateVertical)
   {
      List<RigidBodyTransform> centroidPoses = new ArrayList<>();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      RigidBodyTransform centerOne = new RigidBodyTransform(stairOriginTransform);
      ConvexPolygon2D polygonOne = new ConvexPolygon2D();
      polygonOne.addVertex(lowerLength, width / 2);
      polygonOne.addVertex(lowerLength, -width / 2);
      polygonOne.addVertex(0.0, -width / 2);
      polygonOne.addVertex(0.0, width / 2);
      polygonOne.update();

      RigidBodyTransform centerTwo = new RigidBodyTransform(stairOriginTransform);
      centerTwo.appendTranslation(lowerLength, 0.0, 0.0);
      centerTwo.appendPitchRotation(Math.toRadians(-90.0));
      centerTwo.appendTranslation(height / 2, 0.0, 0.0);
      ConvexPolygon2D polygonTwo = new ConvexPolygon2D();
      polygonTwo.addVertex(height / 2, width / 2);
      polygonTwo.addVertex(height / 2, -width / 2);
      polygonTwo.addVertex(-height / 2, -width / 2);
      polygonTwo.addVertex(-height / 2, width / 2);
      polygonTwo.update();

      RigidBodyTransform centerThr = new RigidBodyTransform(stairOriginTransform);
      centerThr.appendTranslation(lowerLength + upperLength / 2, 0.0, height);
      ConvexPolygon2D polygonThr = new ConvexPolygon2D();
      polygonThr.addVertex(upperLength / 2, width / 2);
      polygonThr.addVertex(upperLength / 2, -width / 2);
      polygonThr.addVertex(-upperLength / 2, -width / 2);
      polygonThr.addVertex(-upperLength / 2, width / 2);
      polygonThr.update();

      centroidPoses.add(centerOne);
      if (generateVertical)
         centroidPoses.add(centerTwo);
      centroidPoses.add(centerThr);
      convexPolygons.add(polygonOne);
      if (generateVertical)
         convexPolygons.add(polygonTwo);
      convexPolygons.add(polygonThr);

      RigidBodyTransform preMultiplier = new RigidBodyTransform(sensorPose);
      preMultiplier.multiply(perturbedTransformer);
      preMultiplier.multiplyInvertOther(sensorPose);
      for (RigidBodyTransform centroid : centroidPoses)
      {
         centroid.preMultiply(preMultiplier);
      }

      RigidBodyTransform sensorPoseToPack = new RigidBodyTransform(sensorPose);
      sensorPoseToPack.multiply(perturbedTransformer);

      return SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPoseToPack, NUMBER_OF_POINTS, convexPolygons,
                                                                                                 centroidPoses);
   }

   public static StereoVisionPointCloudMessage generateMessageSimplePlane(RigidBodyTransform sensorPose, RigidBodyTransform planeOrigin, double length,
                                                                          double width)
   {
      List<RigidBodyTransform> centroidPoses = new ArrayList<>();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      RigidBodyTransform center = new RigidBodyTransform();
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(length / 2, width / 2);
      polygon.addVertex(length / 2, -width / 2);
      polygon.addVertex(-length / 2, -width / 2);
      polygon.addVertex(-length / 2, width / 2);
      polygon.update();

      centroidPoses.add(center);
      convexPolygons.add(polygon);

      for (RigidBodyTransform centroid : centroidPoses)
         centroid.preMultiply(planeOrigin);
      return SimulatedStereoVisionPointCloudMessageFactory.generateStereoVisionPointCloudMessage(sensorPose, NUMBER_OF_POINTS, convexPolygons, centroidPoses);
   }
}
