package us.ihmc.robotEnvironmentAwareness.updaters;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.awt.List;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REASensorDataFilterParametersMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.LineSegment3D;
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
import us.ihmc.ros2.Ros2Node;

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

   private final AtomicReference<Double> octreeResolution;

   private final REAOcTreeBuffer lidarBufferUpdater;
   private final REAOcTreeBuffer stereoVisionBufferUpdater;
   private final REAOcTreeUpdater mainUpdater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

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

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager.submitMessage(REAModuleAPI.RequestEntireModuleState, true);

      preserveOcTreeHistory = reaMessager.createInput(REAModuleAPI.StereoVisionBufferPreservingEnable, false);
      enableStereoBuffer = reaMessager.createInput(REAModuleAPI.StereoVisionBufferEnable, false);
      octreeResolution = reaMessager.createInput(REAModuleAPI.OcTreeResolution, mainUpdater.getMainOctree().getResolution());
      
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      reaMessager.submitMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
      reaMessager.submitMessage(REAModuleAPI.UIStereoVisionShow, true);

      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      reaMessager.submitMessage(REAModuleAPI.UIOcTreeBoundingBoxShow, true);
      BoundingBoxParametersMessage boundingBox = new BoundingBoxParametersMessage();
      boundingBox.maxX = 1.0f;
      boundingBox.minX = -1.0f;
      boundingBox.maxY = 1.0f;
      boundingBox.minY = -0.1f;
      boundingBox.maxZ = 2.0f;
      boundingBox.minZ = 0.0f;
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBox);      
      
      //defailt
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, PlanarRegionSegmentationParameters.parse(
         "search radius: 0.05, max distance from plane: 0.05, maxAngleFromPlane: 0.17453292519943295, minNormalQuality: 0.005"
         + ", min region size: 50, max standard deviation: 0.015, min volumic density: 100000.0"
         ));   
   }

   private void dispatchLidarScanMessage(Subscriber<LidarScanMessage> subscriber)
   {
      LidarScanMessage message = subscriber.takeNextData();
      moduleStateReporter.registerLidarScanMessage(message);
      lidarBufferUpdater.handleLidarScanMessage(message);
      mainUpdater.handleLidarScanMessage(message);
   }

   private void dispatchStereoVisionPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
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

            double stairDistance = stairDistance(planarRegionFeatureUpdater.getPlanarRegionsList());
            
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
    * returns distance to the closest stair, if no stair detected then returns 999.0
    */
   private double stairDistance(PlanarRegionsList planarRegionsList)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0)
         return 999.0;
      
      //params
      final double angleToCameraTolerance = 25.0; //20
      final double angleBetweenPlanesTolerance = 10.0;
      final double XYZTolerance = 0.1;
      
      //variable
      double distance = 999.0;
      
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
            
            //System.out.println(counter++); 
         }
         
         /*
         normals[i] = planarRegion.getNormal();
         
         
         double angle = Math.acos(vector.getZ())*180/Math.PI;
         //System.out.println(angle + ", " + (angle > positiveAngle ? "true" : "false"));
         if(angle > positiveAngle) {
            double D2Distance = planarRegion.distanceToPointByProjectionOntoXYPlane(0.0, 0.0);
            //System.out.println(D2Distance);
            if(D2Distance < positiveD2Distance) {
               distance = planarRegion.getPlaneZGivenXY(0, 0);
               //System.out.println("obstacle at " + distance + " meters");
               distanceList.add(distance);                        
            }
         }  */             
      }
/*
      for(int i = 0; i < normals.length; i++) {
         for(int j = 0; j < normals.length; j++) {
            if(j == i) {
               continue;
            }
            
            
         }
      }   */   

      if(distance != 999.0) {
         System.out.println(distance);         
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
