package us.ihmc.robotEnvironmentAwareness.updaters;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.awt.List;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REASensorDataFilterParametersMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import std_msgs.msg.dds.Float64;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;

/*
 * original class changed for testing purposes
 */
public class LIDARBasedREAModule
{
   private static final String ocTreeTimeReport = "OcTree update took: ";
   private static final String reportOcTreeStateTimeReport = "Reporting OcTree state took: ";
   private static final String planarRegionsTimeReport = "OcTreePlanarRegion update took: ";
   private static final String reportPlanarRegionsStateTimeReport = "Reporting Planar Regions state took: ";

   private final TimeReporter timeReporter = new TimeReporter();

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   protected static final boolean DEBUG = true;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());
   private final RealtimeRos2Node realTimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName() + "RealTime");

   private final AtomicReference<Double> octreeResolution;

   private final REAOcTreeBuffer lidarBufferUpdater;
   private final REAOcTreeBuffer stereoVisionBufferUpdater;
   private final REAOcTreeUpdater mainUpdater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;
   public static final double DEFAULT_DISTANCE_VALUE = 999.0;

   private final REAModuleStateReporter moduleStateReporter;
   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProvider;

   private final AtomicReference<Boolean> clearOcTree;
   private final AtomicReference<Boolean> enableStereoBuffer;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager;

   private final AtomicReference<Boolean> preserveOcTreeHistory;

   private LIDARBasedREAModule(Messager reaMessager, File configurationFile) throws IOException
   {
      this.reaMessager = reaMessager;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);
      lidarBufferUpdater = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager, REAModuleAPI.LidarBufferEnable, true,
                                               REAModuleAPI.LidarBufferOcTreeCapacity, 10000, REAModuleAPI.LidarBufferMessageCapacity, 500,
                                               REAModuleAPI.RequestLidarBuffer, REAModuleAPI.LidarBufferState);
      stereoVisionBufferUpdater = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager, REAModuleAPI.StereoVisionBufferEnable, false,
                                                      REAModuleAPI.StereoVisionBufferOcTreeCapacity, 1000000, REAModuleAPI.StereoVisionBufferMessageCapacity, 1,
                                                      REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState);
      REAOcTreeBuffer[] bufferUpdaters = new REAOcTreeBuffer[] {lidarBufferUpdater, stereoVisionBufferUpdater};
      mainUpdater = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdaters, reaMessager);
      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(reaMessager);

      ROS2Tools.createCallbackSubscription(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan", this::dispatchLidarScanMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, "/ihmc/stereo_vision_point_cloud",
                                           this::dispatchStereoVisionPointCloudMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, subscriberCustomRegionsTopicNameGenerator,
                                           this::dispatchCustomPlanarRegion);
      ROS2Tools.createCallbackSubscription(ros2Node, RequestPlanarRegionsListMessage.class, subscriberTopicNameGenerator,
                                           this::handleRequestPlanarRegionsListMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, REAStateRequestMessage.class, subscriberTopicNameGenerator, this::handleREAStateRequestMessage);
      ROS2Tools.createCallbackSubscription(ros2Node, REASensorDataFilterParametersMessage.class, subscriberTopicNameGenerator,
                                           this::handleREASensorDataFilterParametersMessage);
      
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "mina_v2/knee_height"
                                           , this::handleExoKneeHeight);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "mina_v2/thigh_angle"
                                           , this::handleExoThighAngle);

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> lidarBufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration,
                                        (content) -> planarRegionFeatureUpdater.saveConfiguration(filePropertyHelper));

      planarRegionNetworkProvider = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdater, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);
      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);


      preserveOcTreeHistory = reaMessager.createInput(REAModuleAPI.StereoVisionBufferPreservingEnable, false);
      enableStereoBuffer = reaMessager.createInput(REAModuleAPI.StereoVisionBufferEnable, false);
      octreeResolution = reaMessager.createInput(REAModuleAPI.OcTreeResolution, mainUpdater.getMainOctree().getResolution());
      
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      reaMessager.submitMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
      reaMessager.submitMessage(REAModuleAPI.UIStereoVisionShow, true);

      BoundingBoxParametersMessage boundingBox = new BoundingBoxParametersMessage();
      /*
      //right
      boundingBox.maxX = 0.0f;
      boundingBox.minX = -1.0f;
      boundingBox.maxY = 0.1f;
      boundingBox.minY = -1.0f;
      boundingBox.maxZ = 3.0f;
      boundingBox.minZ = 0.0f;
      */   
      
      //left
      boundingBox.maxX = 0.01f;
      boundingBox.minX = -1.0f;
      boundingBox.maxY = -0.1f;
      boundingBox.minY = 1.0f;
      boundingBox.maxZ = 3.0f;
      boundingBox.minZ = 0.0f;
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBox);  

      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      reaMessager.submitMessage(REAModuleAPI.UIOcTreeBoundingBoxShow, true);
      
      //defailt
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, PlanarRegionSegmentationParameters.parse(
         "search radius: 0.05, max distance from plane: 0.05, maxAngleFromPlane: 0.17453292519943295, minNormalQuality: 0.005"
         + ", min region size: 50, max standard deviation: 0.015, min volumic density: 100000.0"
         ));   

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager.submitMessage(REAModuleAPI.RequestEntireModuleState, true);
   }
   
   private double DISTANCE_CAMERA_GROUND = 0.635;
   private int CAMERA_POSITION = 1;
   private void handleExoKneeHeight(Subscriber<Float64> subscriber) {
      Double value = subscriber.takeNextData().data_;
      if(value == 0.0)
         return;
      
      if(value == 500.0) {
         DISTANCE_CAMERA_GROUND = 500.0;
         return;
      }
      
      switch (CAMERA_POSITION)
      {
         case 1:
            DISTANCE_CAMERA_GROUND = value + Math.sin(THIGH_ANGLE - 0.69001592) * 0.0943; //0.69001592 rad (39.535°) and 0.0943 meters are measurements from exo
            break;
         case 2:
            DISTANCE_CAMERA_GROUND = value + Math.sin(THIGH_ANGLE - 0.265586752) * 0.0631; //0.265586752 rad (15.217°) and 0.0631 meters are measurements from exo
            break;
         default:
            break;
      }
   } 
   
   private double THIGH_ANGLE = 1.5708;
   private double IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = 90.0;
   private double IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = 0.0;
   private void handleExoThighAngle(Subscriber<Float64> subscriber) {
      Double value = subscriber.takeNextData().data_;
      if(value == 0.0 || value == 500.0)
         return;
      
      THIGH_ANGLE = value;
      
      switch (CAMERA_POSITION)
      {
         case 1:
            IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = (180/Math.PI)* (1.5708 - (THIGH_ANGLE - 1.5708)); // 1.5708 rad (90°) is ideal angle between camera view vector(forward) and stair 
            IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - 90;
            break;
         case 2:
            IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = (180/Math.PI)* ((1.5708 + 0.698132) - (THIGH_ANGLE - 1.5708)); // when camera is looking down, where is additional 0.698132 rad (40°) 
            IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - 90;
            break;
         default:
            break;
      }
   }
   
   private void dispatchLidarScanMessage(Subscriber<LidarScanMessage> subscriber)
   {
      LidarScanMessage message = subscriber.takeNextData();
      moduleStateReporter.registerLidarScanMessage(message);
      lidarBufferUpdater.handleLidarScanMessage(message);
      mainUpdater.handleLidarScanMessage(message);
   }

   boolean switchingThingy = true;
   private void dispatchStereoVisionPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();   
      
      /*
      if(switchingThingy) {
         switchingThingy = false;
         RotationScaleMatrix rotation = new RotationScaleMatrix();
         rotation.setEuler(0.0, -IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE*(Math.PI/180), 0.0); 
         Point3D translation = new Point3D(0.0, 0.0, 0.0);
         AffineTransform transform = new AffineTransform(rotation, translation);         
         
         Float floatIGuess = message.getPointCloud();
         for(int i = 0; i < floatIGuess.size(); i+=3) {
            Point3D point = new Point3D(floatIGuess.get(i), floatIGuess.get(i+1), floatIGuess.get(i+2));
            point.applyTransform(transform);
            floatIGuess.set(i, point.getX32());
            floatIGuess.set(i+1, point.getY32());
            floatIGuess.set(i+2, point.getZ32());
         }         
      }
      else {
         switchingThingy = true;
      }
      */
      
      moduleStateReporter.registerStereoVisionPointCloudMessage(message);
      stereoVisionBufferUpdater.handleStereoVisionPointCloudMessage(message);
      mainUpdater.handleStereoVisionPointCloudMessage(message);
   }

   private void dispatchCustomPlanarRegion(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
   }

   private void handleRequestPlanarRegionsListMessage(Subscriber<RequestPlanarRegionsListMessage> subscriber)
   {
      RequestPlanarRegionsListMessage newMessage = subscriber.takeNextData();
      PlanarRegionsRequestType requestType = PlanarRegionsRequestType.fromByte(newMessage.getPlanarRegionsRequestType());
      if (requestType == PlanarRegionsRequestType.CLEAR)
         clearOcTree.set(true);
   }

   private void handleREAStateRequestMessage(Subscriber<REAStateRequestMessage> subscriber)
   {
      REAStateRequestMessage newMessage = subscriber.takeNextData();

      if (newMessage.getRequestResume())
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true);
      else if (newMessage.getRequestPause()) // We guarantee to resume if requested, regardless of the pause request.
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, false);
      if (newMessage.getRequestClear())
         clearOcTree.set(true);
   }

   private void handleREASensorDataFilterParametersMessage(Subscriber<REASensorDataFilterParametersMessage> subscriber)
   {
      REASensorDataFilterParametersMessage newMessage = subscriber.takeNextData();

      if (!newMessage.getBoundingBoxMin().containsNaN() && !newMessage.getBoundingBoxMax().containsNaN())
      {
         BoundingBoxParametersMessage boundingBox = new BoundingBoxParametersMessage();
         boundingBox.getMin().set(newMessage.getBoundingBoxMin());
         boundingBox.getMax().set(newMessage.getBoundingBoxMax());
         reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBox);
      }
      if (newMessage.getSensorMinRange() >= 0.0)
         reaMessager.submitMessage(REAModuleAPI.LidarMinRange, newMessage.getSensorMinRange());
      if (newMessage.getSensorMaxRange() >= 0.0)
         reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, newMessage.getSensorMaxRange());
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      lidarBufferUpdater.loadConfiguration(filePropertyHelper);
      stereoVisionBufferUpdater.loadConfiguration(filePropertyHelper);
      mainUpdater.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdater.loadConfiguration(filePropertyHelper);
   }

   private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      boolean ocTreeUpdateSuccess = true;

      try
      {
         NormalOcTree mainOctree = mainUpdater.getMainOctree();
         if (clearOcTree.getAndSet(false))
         {
            lidarBufferUpdater.clearBuffer();
            stereoVisionBufferUpdater.clearBuffer();
            mainUpdater.clearOcTree();
            planarRegionFeatureUpdater.clearOcTree();
            if (mainOctree.getResolution() != octreeResolution.get())
            {
               lidarBufferUpdater.setOctreeResolution(octreeResolution.get());
               stereoVisionBufferUpdater.setOctreeResolution(octreeResolution.get());
               mainUpdater.initializeReferenceOctree(octreeResolution.get());
            }
         }
         else
         {
            if (enableStereoBuffer.get() && !preserveOcTreeHistory.get())
               mainUpdater.clearOcTree();

            timeReporter.run(mainUpdater::update, ocTreeTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportOcTreeState(mainOctree), reportOcTreeStateTimeReport);

            if (isThreadInterrupted())
               return;

            timeReporter.run(() -> planarRegionFeatureUpdater.update(mainOctree), planarRegionsTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportPlanarRegionsState(planarRegionFeatureUpdater), reportPlanarRegionsStateTimeReport);

            double stairDistance = stairDistance2(planarRegionFeatureUpdater.getPlanarRegionsList());
            if(stairDistance != DEFAULT_DISTANCE_VALUE)
               System.out.println(stairDistance);
               
            /*
            double distance = obstacleDistance(planarRegionFeatureUpdater.getPlanarRegionsList());
            if(distance != DEFAULT_DISTANCE_VALUE)
               System.out.println(distance);
            */            
            
            planarRegionNetworkProvider.update(ocTreeUpdateSuccess);
            planarRegionNetworkProvider.publishCurrentState();
         }

         if (isThreadInterrupted())
            return;
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
         else
         {
            LogTools.error(e.getClass().getSimpleName());
         }
      }

      currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      if (ocTreeUpdateSuccess)
         lastCompleteUpdate.set(currentTime);
   }

   int counter = 0;
   
   /*
    * returns distance to the closest stair, if no stair detected then returns DEFAULT_DISTANCE_VALUE
    * OBSOLETE
    */
   private double stairDistance(PlanarRegionsList planarRegionsList)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0)
         return DEFAULT_DISTANCE_VALUE;
      
      //params
      final double angleToCameraTolerance = 25.0;
      final double angleBetweenPlanesTolerance = 15.0;
      final double XYZTolerance = 0.1;
      
      //variable
      double distance = DEFAULT_DISTANCE_VALUE;

      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal();
         double angleToCamera = Math.acos(normalI.getZ())*180/Math.PI; //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < 180.0 - angleToCameraTolerance)
            continue;
        
         for(int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++) {
            if(j == i)
               continue;
            
            PlanarRegion planarRegionJ = planarRegionsList.getPlanarRegion(j);
            Vector3D normalJ = planarRegionJ.getNormal();
            double angleBetweenPlanes = normalJ.angle(normalI)*180/Math.PI;
            if(angleBetweenPlanes < 90.0 - angleBetweenPlanesTolerance || angleBetweenPlanes > 90.0 + angleBetweenPlanesTolerance)
               continue;           

            double IminX = planarRegionI.getBoundingBox3dInWorld().getMinX();
            double IminY = planarRegionI.getBoundingBox3dInWorld().getMinY();
            double IminZ = planarRegionI.getBoundingBox3dInWorld().getMinZ();

            double ImaxX = planarRegionI.getBoundingBox3dInWorld().getMaxX();
            double ImaxY = planarRegionI.getBoundingBox3dInWorld().getMaxY();
            double ImaxZ = planarRegionI.getBoundingBox3dInWorld().getMaxZ();            

            double JminX = planarRegionJ.getBoundingBox3dInWorld().getMinX();
            double JminY = planarRegionJ.getBoundingBox3dInWorld().getMinY();
            double JminZ = planarRegionJ.getBoundingBox3dInWorld().getMinZ();
            
            double JmaxX = planarRegionJ.getBoundingBox3dInWorld().getMaxX();
            double JmaxY = planarRegionJ.getBoundingBox3dInWorld().getMaxY();
            double JmaxZ = planarRegionJ.getBoundingBox3dInWorld().getMaxZ();
            
            if(Math.abs(ImaxX - JminX) > XYZTolerance) 
               continue;
            
            if(ImaxY < JminY || IminY > JmaxY)
               continue;            
            
            if(distance > JminZ)
               distance = JminZ;            
         }     
      }     
      
      return distance;  
   }
   
   private static double ANGLE_CAMERA_PLANE_TOLERANCE = 10.0;
   private static double MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE = 0.05;  
   /*
    * returns distance to the closest stair, if no stair detected then returns DEFAULT_DISTANCE_VALUE
    * different approach
    */
   private double stairDistance2(PlanarRegionsList planarRegionsList)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0 || DISTANCE_CAMERA_GROUND == 500.0) // 500 means leg in swing
         return DEFAULT_DISTANCE_VALUE;      
            
      //variable
      double distance = DEFAULT_DISTANCE_VALUE;
      
      RotationScaleMatrix rotation = null;
      Point3D translation = null;
      AffineTransform transform = null; 
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal();
         double angleToCamera = Math.acos(normalI.getZ())*(180/Math.PI); //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - ANGLE_CAMERA_PLANE_TOLERANCE || angleToCamera > IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE + ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;

         double angleToGround = Math.acos(normalI.getX())*(180/Math.PI); //simplified for ground vector (1, 0, 0)
         if(angleToGround < Math.abs(IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE) - ANGLE_CAMERA_PLANE_TOLERANCE || angleToGround > Math.abs(IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE) + ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;
         
         if(rotation == null) {
            rotation = new RotationScaleMatrix();
            rotation.setEuler(0.0, -IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE*(Math.PI/180), 0.0);
            translation = new Point3D(0.0, 0.0, 0.0);
            transform = new AffineTransform(rotation, translation); 
         }
         
         Point3D minPoint = (Point3D) planarRegionI.getBoundingBox3dInWorld().getMinPoint();
         if(planarRegionI.getNormal().getZ() > 0) 
            minPoint.setX(planarRegionI.getBoundingBox3dInWorld().getMaxPoint().getX());
         minPoint.applyTransform(transform);
         
         double pointToGround = minPoint.getX() * -1.0; 
         if(pointToGround > DISTANCE_CAMERA_GROUND - MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE && pointToGround < DISTANCE_CAMERA_GROUND + MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE)
            continue;
         
         if(distance > minPoint.getZ())
            distance =  minPoint.getZ();
         
         double stairHeight = DISTANCE_CAMERA_GROUND - pointToGround;
      }     
      
      return distance;        
   }

   /*
    * returns DEFAULT_DISTANCE_VALUE if there is no obstacle otherwise return distance to obstacle
    * OBSOLETE
    */
   private double obstacleDistance(PlanarRegionsList planarRegionsList) {                    
      //variables
      double distance = DEFAULT_DISTANCE_VALUE;  
      LinkedList<Double> distanceList = new LinkedList<Double>();
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         Vector3D vector = planarRegion.getNormal();
         double angle = Math.acos(vector.getZ())*180/Math.PI;
         if(angle > 180.0 - 20.0) {
            double D2Distance = planarRegion.distanceToPointByProjectionOntoXYPlane(0.0, 0.0);
            if(D2Distance < 0.5) {
               distance = planarRegion.getPlaneZGivenXY(0, 0);
               distanceList.add(distance);                        
            }
         }               
      }
      
      for(int i = 0; i < distanceList.size() -1; i++){
         if(distanceList.get(i) < distance) {
            distance = distanceList.get(i);
         }
      }
      return distance;   
   }

   
   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start() throws IOException
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(lidarBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      LogTools.info("REA Module is going down.");

      reaMessager.closeMessager();
      ros2Node.destroy();
      realTimeRos2Node.destroy();

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   public static LIDARBasedREAModule createRemoteModule(String configurationFilePath) throws Exception
   {
      KryoMessager server = KryoMessager.createTCPServer(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                         REACommunicationProperties.getPrivateNetClassList());
      server.setAllowSelfSubmit(true);
      server.startMessager();
      return new LIDARBasedREAModule(server, new File(configurationFilePath));
   }

   public static LIDARBasedREAModule createIntraprocessModule(String configurationFilePath) throws Exception
   {
      KryoMessager messager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();

      File configurationFile = new File(configurationFilePath);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      return new LIDARBasedREAModule(messager, configurationFile);
   }
}
