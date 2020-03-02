package us.ihmc.robotEnvironmentAwareness.slam;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;

public class GridNettingSLAM extends SLAMBasics
{
   public static final boolean DEBUG = true;
   public GridNettingSLAM(double octreeResolution)
   {
      super(octreeResolution);
   }

   private void insertNewPointCloud(SLAMFrame frame)
   {
      Point3DReadOnly[] pointCloud = frame.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = getLatestFrame().getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation()));

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);
   }
   
   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      super.addKeyFrame(pointCloudMessage);

      SLAMFrame firstFrame = getLatestFrame();

      insertNewPointCloud(firstFrame);
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      boolean success = super.addFrame(pointCloudMessage);

      if (success)
      {
         SLAMFrame newFrame = getLatestFrame();
         insertNewPointCloud(newFrame);
      }

      return success;
   }
   
   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      return new RigidBodyTransform();
   }
}
