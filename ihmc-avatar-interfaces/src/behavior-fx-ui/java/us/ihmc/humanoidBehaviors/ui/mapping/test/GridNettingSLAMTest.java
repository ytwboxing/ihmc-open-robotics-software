package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.GridNettingSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;

public class GridNettingSLAMTest
{

   @Test
   public void testComputeGridNettingLineSegments()
   {
      String stereoPath = "E:\\Data\\20200213_Round_1\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      GridNettingSLAM slam = new GridNettingSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      //int frameIDToTest = 17;
      int frameIDToTest = 13;

      for (int i = 0; i < frameIDToTest; i++)
      {
         slam.addKeyFrame(messages.get(i));
         slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
         slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);
      }

      SLAMFrame frameToTest = new SLAMFrame(slam.getLatestFrame(), messages.get(frameIDToTest));
      slamViewer.addSensorPose(frameToTest.getSensorPose(), Color.BLUE);
      slamViewer.addPointCloud(frameToTest.getPointCloud(), Color.BLUE);

      double windowMargin = 0.01;
      ConvexPolygon2D mapWindowFromSensorPose = SLAMTools.computeWindowForMapToSensorPose(slam.getOctree(), windowMargin,
                                                                                          frameToTest.getInitialSensorPoseToWorld());

      Point3DReadOnly[] newPointCloudToSensorPose = frameToTest.getOriginalPointCloudToSensorPose();
      double maxX = Double.NEGATIVE_INFINITY;
      double minX = Double.POSITIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;

      boolean[] isInPreviousView = new boolean[newPointCloudToSensorPose.length];
      int numberOfPointsInWindow = 0;
      for (int i = 0; i < newPointCloudToSensorPose.length; i++)
      {
         Point3DReadOnly point = newPointCloudToSensorPose[i];
         isInPreviousView[i] = false;
         if (mapWindowFromSensorPose.isPointInside(point.getX(), point.getY(), -windowMargin))
         {
            isInPreviousView[i] = true;
            numberOfPointsInWindow++;
            
            if (maxX < point.getX())
               maxX = point.getX();
            if (minX > point.getX())
               minX = point.getX();
            if (maxY < point.getY())
               maxY = point.getY();
            if (minY > point.getY())
               minY = point.getY();
         }
      }

      System.out.println("numberOfPointsInWindow " + numberOfPointsInWindow + " (" + newPointCloudToSensorPose.length + ")");
      Point3D[] pointsInPreviousWindowToSensorPose = new Point3D[numberOfPointsInWindow];

      int indexOfPointsInWindow = 0;
      for (int i = 0; i < newPointCloudToSensorPose.length; i++)
      {
         if (isInPreviousView[i])
         {
            pointsInPreviousWindowToSensorPose[indexOfPointsInWindow] = new Point3D(newPointCloudToSensorPose[i]);
            indexOfPointsInWindow++;
         }
      }
      Point3D[] pointsInPreviousWindowToWorld = SLAMTools.createConvertedPointsToWorld(frameToTest.getInitialSensorPoseToWorld(),
                                                                                       pointsInPreviousWindowToSensorPose);
      slamViewer.addPointCloud(pointsInPreviousWindowToWorld, Color.RED);

      double lineThickness = 0.01 * 2;
      List<List<Point3D>> pointsOnVerticalLines = new ArrayList<>();
      int gridResolution = 6;
      double[] verticalGridLines = generateInnerGridLines(minX, maxX, gridResolution);
      double[] horizontalGridLines = generateInnerGridLines(minY, maxY, gridResolution);
      for (int i = 0; i < (gridResolution - 1) * 2; i++)
      {
         List<Point3D> pointsOnLine = new ArrayList<>();
         pointsOnVerticalLines.add(pointsOnLine);
      }

      for (int i = 0; i < pointsInPreviousWindowToSensorPose.length; i++)
      {
         double x = pointsInPreviousWindowToSensorPose[i].getX();
         double y = pointsInPreviousWindowToSensorPose[i].getY();

         for (int j = 0; j < verticalGridLines.length; j++)
         {
            if (x < verticalGridLines[j] + lineThickness / 2 && x > verticalGridLines[j] - lineThickness / 2)
            {
               pointsOnVerticalLines.get(j).add(pointsInPreviousWindowToSensorPose[i]);
               break;
            }
         }
         for (int j = 0; j < horizontalGridLines.length; j++)
         {
            if (y < horizontalGridLines[j] + lineThickness / 2 && y > horizontalGridLines[j] - lineThickness / 2)
            {
               pointsOnVerticalLines.get(j + gridResolution - 1).add(pointsInPreviousWindowToSensorPose[i]);
               break;
            }
         }
      }

      for (int i = 0; i < gridResolution - 1; i++)
      {
         Point3D[] pointsOnLineToSensor = new Point3D[pointsOnVerticalLines.get(i).size()];
         for (int j = 0; j < pointsOnLineToSensor.length; j++)
         {
            pointsOnLineToSensor[j] = pointsOnVerticalLines.get(i).get(j);
         }
         Point3D[] pointsOnLineToWorld = SLAMTools.createConvertedPointsToWorld(frameToTest.getInitialSensorPoseToWorld(), pointsOnLineToSensor);
         System.out.println("pointsOnLineToWorld "+pointsOnLineToWorld.length);
         slamViewer.addPointCloud(pointsOnLineToWorld, Color.BLACK);
      }
      
      for (int i = gridResolution - 1; i < pointsOnVerticalLines.size(); i++)
      {
         Point3D[] pointsOnLineToSensor = new Point3D[pointsOnVerticalLines.get(i).size()];
         for (int j = 0; j < pointsOnLineToSensor.length; j++)
         {
            pointsOnLineToSensor[j] = pointsOnVerticalLines.get(i).get(j);
         }
         Point3D[] pointsOnLineToWorld = SLAMTools.createConvertedPointsToWorld(frameToTest.getInitialSensorPoseToWorld(), pointsOnLineToSensor);
         System.out.println("pointsOnLineToWorld "+pointsOnLineToWorld.length);
         slamViewer.addPointCloud(pointsOnLineToWorld, Color.YELLOW);
      }

      slamViewer.start("testComputeGridNettingLineSegments");
      ThreadTools.sleepForever();
   }

   private double[] generateInnerGridLines(double min, double max, int resolution)
   {
      double[] gridLines = new double[(resolution - 1)];
      double delta = (max - min) / resolution;
      for (int i = 0; i < resolution - 1; i++)
      {
         gridLines[i] = min + delta * (i + 1);
      }
      return gridLines;
   }

   @Test
   public void testFindBadFrame()
   {
      //String stereoPath = "E:\\Data\\20200218_simple\\PointCloud\\"; // 11, 12 has gap with 10.
      String stereoPath = "E:\\Data\\20200213_Round_1\\PointCloud\\";
      // 17 has gap with 16, 15.
      // 7 has gap with 6.
      // 8 is very bad frame.
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      GridNettingSLAM slam = new GridNettingSLAM(octreeResolution);
      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messages.get(0));
      slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);

      for (int i = 1; i < 20; i++)
      {
         slam.addFrame(messages.get(i));
         slamViewer.addSensorPose(slam.getLatestFrame().getSensorPose(), Color.GREEN);
         slamViewer.addPointCloud(slam.getLatestFrame().getPointCloud(), Color.GREEN);
      }

      slamViewer.start("testFindBadFrame");
      ThreadTools.sleepForever();
   }
}
