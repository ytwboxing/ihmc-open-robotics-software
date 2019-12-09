package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class IhmcSLAMTest
{
   @Test
   public void testViewer()
   {
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));

      viewer.addStereoMessage(messagesFromFile.get(20), Color.GREEN, Color.GREEN);
      viewer.addStereoMessage(messagesFromFile.get(25), Color.BLUE, Color.BLUE);
      viewer.start("testViewer");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedPointCloudFrameForStair()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPose = new RigidBodyTransform();
      sensorPose.setTranslation(0.0, 0.0, 1.0);
      sensorPose.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOrigin = new RigidBodyTransform();
      RigidBodyTransform perturb = new RigidBodyTransform();

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPose, satirOrigin, stairLength,
                                                                                                                       stairLength, stairWidth, stairHeight,
                                                                                                                       perturb, true);
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(message);
      viewer.start("testSimulatedPointCloudFrameForStair");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedContinuousFramesForStair()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginOne = new RigidBodyTransform();
      RigidBodyTransform perturbOne = new RigidBodyTransform();

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne, satirOriginOne,
                                                                                                                          stairLength, stairLength, stairWidth,
                                                                                                                          stairHeight, perturbOne, true);

      double movingForward = 0.1;
      double movingUpward = 0.05;
      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, 1.0 + movingUpward);
      sensorPoseTwo.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginTwo = new RigidBodyTransform(satirOriginOne);
      satirOriginTwo.appendTranslation(movingForward, 0.0, 0.0);
      RigidBodyTransform perturbTwo = new RigidBodyTransform();

      StereoVisionPointCloudMessage messageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, satirOriginTwo,
                                                                                                                          stairLength - movingForward,
                                                                                                                          stairLength + movingForward,
                                                                                                                          stairWidth, stairHeight, perturbTwo,
                                                                                                                          true);
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(messageOne, Color.BLUE);
      viewer.addStereoMessage(messageTwo, Color.GREEN);
      viewer.start("testSimulatedContinuousFramesForStair");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimulatedBadFrame()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginOne = new RigidBodyTransform();
      RigidBodyTransform perturbOne = new RigidBodyTransform();

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne, satirOriginOne,
                                                                                                                          stairLength, stairLength, stairWidth,
                                                                                                                          stairHeight, perturbOne, true);

      double movingForward = 0.1;
      double movingUpward = 0.05;
      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, 1.0 + movingUpward);
      sensorPoseTwo.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginTwo = new RigidBodyTransform(satirOriginOne);
      satirOriginTwo.appendTranslation(movingForward, 0.0, 0.0);
      RigidBodyTransform perturbTwo = new RigidBodyTransform();

      double translationX = -0.02;
      double translationY = 0.03;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(3.0);
      perturbTwo.setTranslation(translationX, translationY, translationZ);
      perturbTwo.appendPitchRotation(rotateY);

      StereoVisionPointCloudMessage messageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, satirOriginTwo,
                                                                                                                          stairLength - movingForward,
                                                                                                                          stairLength + movingForward,
                                                                                                                          stairWidth, stairHeight, perturbTwo,
                                                                                                                          true);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addStereoMessage(messageOne, Color.BLUE);
      viewer.addStereoMessage(messageTwo, Color.GREEN);
      viewer.start("testSimulatedBadFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testPlanarRegionsForSimulatedPointCloudFrame()
   {
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPose = new RigidBodyTransform();
      sensorPose.setTranslation(0.0, 0.0, 1.0);
      sensorPose.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOrigin = new RigidBodyTransform();
      RigidBodyTransform perturb = new RigidBodyTransform();

      StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPose, satirOrigin, stairLength,
                                                                                                                       stairLength, stairWidth, stairHeight,
                                                                                                                       perturb, true);

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      Point3DReadOnly[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
      RigidBodyTransformReadOnly sensorPoseFromMessage = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPoseFromMessage.getTranslation(),
                                                                                               octreeResolution, planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPlanarRegions(planarRegionsMap);
      viewer.start("testPlanarRegionsForSimulatedPointCloudFrame");

      ThreadTools.sleepForever();
   }

   @Test
   public void testPlanarRegionsForFlatGround()
   {
      double groundWidth = 1.5;
      double stairLength = 0.3;
      int numberOfMessages = 10;

      double octreeResolution = 0.02;
      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      List<Point3DReadOnly[]> pointCloudMap = new ArrayList<>();

      RigidBodyTransform firstSensorPose = new RigidBodyTransform();
      firstSensorPose.setTranslation(0.0, 0.0, 1.0);
      RigidBodyTransform firstOrigin = new RigidBodyTransform();
      StereoVisionPointCloudMessage firstMessage = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimplePlane(firstSensorPose, firstOrigin,
                                                                                                                            stairLength, groundWidth);
      Point3DReadOnly[] firstPointCloud = IhmcSLAMTools.extractPointsFromMessage(firstMessage);
      RigidBodyTransformReadOnly firstSensorPoseFromMessage = IhmcSLAMTools.extractSensorPoseFromMessage(firstMessage);
      pointCloudMap.add(firstPointCloud);

      List<PlanarRegionSegmentationRawData> firstRawData = IhmcSLAMTools.computePlanarRegionRawData(firstPointCloud,
                                                                                                    firstSensorPoseFromMessage.getTranslation(),
                                                                                                    octreeResolution, planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(firstRawData, concaveHullFactoryParameters, polygonizerParameters);

      for (int i = 1; i < numberOfMessages; i++)
      {
         RigidBodyTransform sensorPose = new RigidBodyTransform(firstSensorPose);
         sensorPose.appendTranslation(i * stairLength, 0.0, 0.0);
         RigidBodyTransform origin = new RigidBodyTransform(firstOrigin);
         origin.appendTranslation(i * stairLength, 0.0, 0.0);

         StereoVisionPointCloudMessage message = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimplePlane(sensorPose, origin, stairLength,
                                                                                                                          groundWidth);

         Point3DReadOnly[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
         RigidBodyTransformReadOnly sensorPoseFromMessage = IhmcSLAMTools.extractSensorPoseFromMessage(message);
         pointCloudMap.add(pointCloud);

         List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(pointCloud, sensorPoseFromMessage.getTranslation(),
                                                                                                  octreeResolution, planarRegionSegmentationParameters);

         planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                                polygonizerParameters);
      }

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPlanarRegions(planarRegionsMap);
      for (int i = 0; i < pointCloudMap.size(); i++)
      {
         int redScaler = (int) (0xFF * (1 - (double) i / pointCloudMap.size()));
         int blueScaler = (int) (0xFF * ((double) i / pointCloudMap.size()));
         Color color = Color.rgb(redScaler, 0, blueScaler);
         viewer.addPointCloud(pointCloudMap.get(i), color);
      }

      viewer.start("testPlanarRegionsForFlatGround");

      ThreadTools.sleepForever();
   }

   @Test
   public void testInverseInterpolation()
   {
      double octreeResolution = 0.02;

      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginOne = new RigidBodyTransform();
      RigidBodyTransform perturbOne = new RigidBodyTransform();

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne, satirOriginOne,
                                                                                                                          stairLength, stairLength, stairWidth,
                                                                                                                          stairHeight, perturbOne, true);

      double movingForward = 0.1;
      double movingUpward = 0.05;
      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, 1.0 + movingUpward);
      sensorPoseTwo.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginTwo = new RigidBodyTransform(satirOriginOne);
      satirOriginTwo.appendTranslation(movingForward, 0.0, 0.0);
      RigidBodyTransform perturbTwo = createRandomDriftedTransform(new Random(0612L), 0.05, 5.0);
      perturbTwo.setIdentity();
      double translationX = -0.02;
      double translationY = 0.03;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(3.0);
      perturbTwo.setTranslation(translationX, translationY, translationZ);
      perturbTwo.appendPitchRotation(rotateY);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, satirOriginTwo,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 stairWidth, stairHeight,
                                                                                                                                 perturbTwo, true);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      NormalOcTree overlappedOctreeNode = driftedFrameTwo.getOctreeNodesInPreviousView();

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(overlappedOctreeNode, Color.GREEN, octreeResolution);
      viewer.start("testInverseInterpolation");

      ThreadTools.sleepForever();
   }

   @Test
   public void testDetectingSimilarPlanarRegions()
   {
      double octreeResolution = 0.02;

      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginOne = new RigidBodyTransform();
      RigidBodyTransform perturbOne = new RigidBodyTransform();

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne, satirOriginOne,
                                                                                                                          stairLength, stairLength, stairWidth,
                                                                                                                          stairHeight, perturbOne, true);

      double movingForward = 0.1;
      double movingUpward = 0.05;
      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, 1.0 + movingUpward);
      sensorPoseTwo.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginTwo = new RigidBodyTransform(satirOriginOne);
      satirOriginTwo.appendTranslation(movingForward, 0.0, 0.0);
      RigidBodyTransform perturbTwo = createRandomDriftedTransform(new Random(0612L), 0.05, 5.0);
      perturbTwo.setIdentity();
      double translationX = -0.02;
      double translationY = 0.03;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(3.0);
      perturbTwo.setTranslation(translationX, translationY, translationZ);
      perturbTwo.appendPitchRotation(rotateY);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, satirOriginTwo,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 stairWidth, stairHeight,
                                                                                                                                 perturbTwo, true);

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frameOne.getPointCloud(),
                                                                                               frameOne.getSensorPose().getTranslation(), octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
      System.out.println("planarRegionsMap " + planarRegionsMap.getNumberOfPlanarRegions());

      double validRatio = 0.0;
      double maximumDistance = 0.1;
      double maximumAngle = Math.toRadians(30.0);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      driftedFrameTwo.computeMergeableSurfaceElements(planarRegionsMap, octreeResolution, validRatio, maximumDistance, maximumAngle);
      List<IhmcSurfaceElement> surfaceElements = driftedFrameTwo.getMergeableSurfaceElements();

      List<IhmcSurfaceElement> groupOne = new ArrayList<>();
      List<IhmcSurfaceElement> groupTwo = new ArrayList<>();
      for (IhmcSurfaceElement element : surfaceElements)
      {
         if (element.getMergeablePlanarRegionId() == planarRegionsMap.getPlanarRegion(0).getRegionId())
            groupOne.add(element);
         if (element.getMergeablePlanarRegionId() == planarRegionsMap.getPlanarRegion(1).getRegionId())
            groupTwo.add(element);
      }

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(groupOne, Color.CORAL);
      viewer.addOctree(groupTwo, Color.BLACK);

      viewer.start("testDetectingSimilarPlanarRegions");

      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimization()
   {
      double octreeResolution = 0.02;

      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginOne = new RigidBodyTransform();
      RigidBodyTransform perturbOne = new RigidBodyTransform();

      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne, satirOriginOne,
                                                                                                                          stairLength, stairLength, stairWidth,
                                                                                                                          stairHeight, perturbOne, false);

      double movingForward = 0.1;
      double movingUpward = 0.05;
      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, 1.0 + movingUpward);
      sensorPoseTwo.appendPitchRotation(Math.toRadians(70.0 + 90.0));
      RigidBodyTransform satirOriginTwo = new RigidBodyTransform(satirOriginOne);
      satirOriginTwo.appendTranslation(movingForward, 0.0, 0.0);
      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.05, 5.0);
      randomTransformer.setIdentity();
      double translationX = -0.02;
      double translationY = 0.03;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(3.0);
      randomTransformer.setTranslation(translationX, translationY, translationZ);
      randomTransformer.appendPitchRotation(rotateY);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, satirOriginTwo,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 stairWidth, stairHeight,
                                                                                                                                 randomTransformer, false);

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMinRegionSize(200);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.02);

      IhmcSLAMFrame frameOne = new IhmcSLAMFrame(messageOne);
      IhmcSLAMFrame driftedFrameTwo = new IhmcSLAMFrame(frameOne, driftedMessageTwo);

      List<PlanarRegionSegmentationRawData> rawData = IhmcSLAMTools.computePlanarRegionRawData(frameOne.getPointCloud(),
                                                                                               frameOne.getSensorPose().getTranslation(), octreeResolution,
                                                                                               planarRegionSegmentationParameters);
      PlanarRegionsList planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      double validRatio = 0.0;
      double maximumDistance = 0.1;
      double maximumAngle = Math.toRadians(30.0);
      driftedFrameTwo.computeOctreeInPreviousView(octreeResolution);
      driftedFrameTwo.computeMergeableSurfaceElements(planarRegionsMap, octreeResolution, validRatio, maximumDistance, maximumAngle);
      List<IhmcSurfaceElement> surfaceElements = driftedFrameTwo.getMergeableSurfaceElements();

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addSensorPose(frameOne.getSensorPose(), Color.BLUE);
      viewer.addSensorPose(driftedFrameTwo.getSensorPose(), Color.YELLOW);
      viewer.addPointCloud(frameOne.getPointCloud(), Color.BLUE);
      viewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      viewer.addOctree(surfaceElements, Color.CORAL);
      viewer.start("testOptimization");

      IhmcSLAMViewer localViewer = new IhmcSLAMViewer();
      localViewer.addPointCloud(frameOne.getPointCloud(), Color.RED);
      localViewer.addPointCloud(driftedFrameTwo.getPointCloud(), Color.YELLOW);
      localViewer.addOctree(surfaceElements, Color.CORAL);
      localViewer.addPlanarRegions(planarRegionsMap);
      PreMultiplierOptimizerCostFunction function = new PreMultiplierOptimizerCostFunction(surfaceElements, driftedFrameTwo.getInitialSensorPoseToWorld());
      GradientDescentModule optimizer = new GradientDescentModule(function, IhmcSLAM.initialQuery);

      int maxIterations = 200;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.1;
      double optimizerPerturbationSize = 0.0001;

      optimizer.setInputLowerLimit(IhmcSLAM.lowerLimit);
      optimizer.setInputUpperLimit(IhmcSLAM.upperLimit);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);
      optimizer.setReducingStepSizeRatio(2);

      int run = optimizer.run();
      System.out.println(run + " " + function.getQuery(IhmcSLAM.initialQuery) + " " + optimizer.getOptimalQuery() + " " + optimizer.getComputationTime());
      TDoubleArrayList optimalInput = optimizer.getOptimalInput();
      System.out.println(optimalInput.get(0) + " " + optimalInput.get(1) + " " + optimalInput.get(2) + " " + optimalInput.get(3));

      RigidBodyTransform slamTransformer = new RigidBodyTransform();
      function.convertToSensorPoseMultiplier(optimalInput, slamTransformer);
      System.out.println("slam transformer");
      System.out.println(slamTransformer);

      System.out.println("randomTransformer");
      System.out.println(randomTransformer);

      driftedFrameTwo.updateSLAM(slamTransformer);

      System.out.println("Test Result : " + slamTransformer.geometricallyEquals(randomTransformer, 0.05));
      System.out.println("Position Diff    : " + slamTransformer.getTranslation().geometricallyEquals(randomTransformer.getTranslation(), 0.05));
      System.out.println("Orientation Diff : " + Math.toRadians(slamTransformer.getRotation().distance(randomTransformer.getRotation())) + " deg.");

      Color color = Color.rgb(0, 255, 0);
      localViewer.addPointCloud(driftedFrameTwo.getPointCloud(), color);
      localViewer.start("iteration ");

      ThreadTools.sleepForever();
   }

   @Test
   public void testTransformer()
   {
      RigidBodyTransform transformer = new RigidBodyTransform();
      transformer.setTranslation(0.1, 0.0, 0.0);
      transformer.appendYawRotation(Math.toRadians(30.0));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();
      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(1.0, 0.0, 1.0);
      sensorPoseOne.appendPitchRotation(Math.toRadians(70.0 + 90.0));

      slamViewer.addSensorPose(sensorPoseOne, Color.RED);

      RigidBodyTransform pointToSensorPose = new RigidBodyTransform();
      pointToSensorPose.setTranslation(0.1, 0.1, 1.0);
      pointToSensorPose.preMultiply(sensorPoseOne);
      slamViewer.addSensorPose(pointToSensorPose, Color.ORANGE);

      sensorPoseOne.preMultiply(transformer);
      pointToSensorPose.preMultiply(transformer);
      slamViewer.addSensorPose(sensorPoseOne, Color.BLACK);
      slamViewer.addSensorPose(pointToSensorPose, Color.DARKORANGE);

      slamViewer.start("testTransformer");

      ThreadTools.sleepForever();
   }

   @Test
   public void testStairAndFlatAndComplex()
   {
      String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
      String planarRegionsPath = "E:\\Data\\SimpleArea3\\20191127_222138_PlanarRegion\\";

      boolean doNaiveSLAM = false;
      boolean showLidarPlanarRegions = false;

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      System.out.println("number of messages " + messages.size());

      IhmcSLAM slam = new IhmcSLAM(doNaiveSLAM);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
         slam.addFrame(messages.get(i));

      if (doNaiveSLAM)
         slam.doNaiveSLAM();

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.BLACK);
         slamViewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         //slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         int redScaler = (int) (0xFF * (1 - (double) i / slam.getPointCloudMap().size()));
         int blueScaler = (int) (0xFF * ((double) i / slam.getPointCloudMap().size()));
         Color color = Color.rgb(redScaler, 0, blueScaler);
         slamViewer.addSensorPose(slam.getSensorPoses().get(i), color);
      }

      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());
      if (showLidarPlanarRegions)
      {
         PlanarRegionsList importPlanarRegionData = PlanarRegionFileTools.importPlanarRegionData(new File(planarRegionsPath));
         for (int i = 0; i < importPlanarRegionData.getNumberOfPlanarRegions(); i++)
         {
            importPlanarRegionData.getPlanarRegion(i).setRegionId(0xFF0000);
         }
         slamViewer.addPlanarRegions(importPlanarRegionData);
      }

      List<List<IhmcSurfaceElement>> allSurfaceElements = slam.allSurfaceElements;
      int size = allSurfaceElements.size();
      for (int i = 0; i < size; i++)
      {
         int redScaler = (int) (0xFF * (1 - (double) i / size));
         int blueScaler = (int) (0xFF * ((double) i / size));
         Color color = Color.rgb(redScaler, 0, blueScaler);
         List<IhmcSurfaceElement> surfaceElements = allSurfaceElements.get(i);
         slamViewer.addOctree(surfaceElements, color);
      }

      slamViewer.start("EndToEnd");

      ThreadTools.sleepForever();
   }

   @Test
   public void testEndToEnd()
   {

   }

   private RigidBodyTransform createRandomDriftedTransform(Random random, double positionBound, double angleBoundDegree)
   {
      RigidBodyTransform randomTransform = new RigidBodyTransform();
      int positionAxis = random.nextInt(2);
      int angleAxis = random.nextInt(2);

      double randomAngle = 2 * Math.toRadians(angleBoundDegree) * (random.nextDouble() - 0.5);
      if (angleAxis == 0)
         randomTransform.appendRollRotation(randomAngle);
      if (angleAxis == 1)
         randomTransform.appendPitchRotation(randomAngle);
      else
         randomTransform.appendYawRotation(randomAngle);

      double randomTranslation = 2 * positionBound * (random.nextDouble() - 0.5);
      if (positionAxis == 0)
         randomTransform.appendTranslation(randomTranslation, 0.0, 0.0);
      if (positionAxis == 1)
         randomTransform.appendTranslation(0.0, randomTranslation, 0.0);
      else
         randomTransform.appendTranslation(0.0, 0.0, randomTranslation);

      System.out.println("positionAxis " + positionAxis + " " + randomTranslation);
      System.out.println("angleAxis " + angleAxis + " " + Math.toDegrees(randomAngle));

      return randomTransform;
   }
}
