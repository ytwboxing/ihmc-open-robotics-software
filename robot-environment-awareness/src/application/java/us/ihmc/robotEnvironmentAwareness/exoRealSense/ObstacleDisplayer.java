package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeBuffer;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeUpdater;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

/*
 * main class that connect realsense D435 (RealSenseBridgeRos2), runs point cloud through planar region finder (most of this class), evaluate obstacles (function obstacleDistance) and present them to hololense (UDPDataSender) 
 */
public class ObstacleDisplayer
{   
   //variables
   private static Ros2Node ros2Node;

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double DEFAULT_OCTREE_RESOLUTION = 0.2;

   protected static final boolean DEBUG = true;
   
   private final REAOcTreeBuffer stereoVisionBufferUpdaterLeft;
   private final REAOcTreeUpdater mainUpdaterLeft; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdaterLeft;
   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProviderLeft;   

   private final REAOcTreeBuffer stereoVisionBufferUpdaterRight;
   private final REAOcTreeUpdater mainUpdaterRight; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdaterRight;
   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProviderRight;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager;  
   
   private static UDPDataSender sender;
   
   //functions
   public static void main(String[] args)
   {
      try {
         ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());
         //connection to realsense D415   
         //new RealSenseBridgeRos2("http://192.168.137.2:11311", "/camera/depth/color/points", ros2Node, ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class), 200000);        
         new RealSenseBridgeRos2("http://192.168.137.2:11311"
                                 , "/cam_1/depth/color/points"
                                 , ros2Node
                                 , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Left"
                                 , 200000);   
         
         new RealSenseBridgeRos2("http://192.168.137.2:11311"
                                 , "/cam_2/depth/color/points"
                                 , ros2Node
                                 , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Right"
                                 , 200001);       
         
         ObstacleDisplayer module = ObstacleDisplayer.createIntraprocessModule();
         
         sender = new UDPDataSender("192.168.0.11", 6669);
         
         module.start();         
      }
      catch (Exception ex) {
         ex.printStackTrace();         
      }
   }
      
   private ObstacleDisplayer(Messager reaMessager, File configurationFile) throws IOException
   {
      this.reaMessager = reaMessager;

      stereoVisionBufferUpdaterLeft = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager, REAModuleAPI.StereoVisionBufferEnable, false,
                                                      REAModuleAPI.StereoVisionBufferOcTreeCapacity, 1000000, REAModuleAPI.StereoVisionBufferMessageCapacity, 1,
                                                      REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState);
      stereoVisionBufferUpdaterRight = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager, REAModuleAPI.StereoVisionBufferEnable, false,
                                                          REAModuleAPI.StereoVisionBufferOcTreeCapacity, 1000000, REAModuleAPI.StereoVisionBufferMessageCapacity, 1,
                                                          REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState);
      REAOcTreeBuffer[] bufferUpdatersLeft = new REAOcTreeBuffer[] { stereoVisionBufferUpdaterLeft};
      REAOcTreeBuffer[] bufferUpdatersRight = new REAOcTreeBuffer[] { stereoVisionBufferUpdaterRight};
      mainUpdaterLeft = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdatersLeft, reaMessager);
      mainUpdaterRight = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdatersRight, reaMessager);
      planarRegionFeatureUpdaterLeft = new REAPlanarRegionFeatureUpdater(reaMessager);
      planarRegionFeatureUpdaterRight = new REAPlanarRegionFeatureUpdater(reaMessager);

      ROS2Tools.createCallbackSubscription(ros2Node
                                           , StereoVisionPointCloudMessage.class
                                           , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Left"
                                           , this::dispatchStereoVisionPointCloudMessageLeft);
      ROS2Tools.createCallbackSubscription(ros2Node
                                           , StereoVisionPointCloudMessage.class
                                           , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Right"
                                           , this::dispatchStereoVisionPointCloudMessageRight);

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdaterLeft.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdaterLeft.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration, (content) -> planarRegionFeatureUpdaterLeft.saveConfiguration(filePropertyHelper));
      
      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdaterRight.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdaterRight.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration, (content) -> planarRegionFeatureUpdaterRight.saveConfiguration(filePropertyHelper));
      
      planarRegionNetworkProviderLeft = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdaterLeft, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);
      planarRegionNetworkProviderRight = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdaterRight, ros2Node, publisherTopicNameGenerator,
                                                                                 subscriberTopicNameGenerator);

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager.submitMessage(REAModuleAPI.RequestEntireModuleState, true); 
      
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      reaMessager.submitMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
      
      //reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, PlanarRegionSegmentationParameters.parse("lala")); //todo JOBY here I can send new params and tweak planar region
            
   }

   private void dispatchStereoVisionPointCloudMessageLeft(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      stereoVisionBufferUpdaterLeft.handleStereoVisionPointCloudMessage(message);      
      mainUpdaterLeft.handleStereoVisionPointCloudMessage(message);
   }
   
   private void dispatchStereoVisionPointCloudMessageRight(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      stereoVisionBufferUpdaterRight.handleStereoVisionPointCloudMessage(message);      
      mainUpdaterRight.handleStereoVisionPointCloudMessage(message);
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      stereoVisionBufferUpdaterLeft.loadConfiguration(filePropertyHelper);
      mainUpdaterLeft.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdaterLeft.loadConfiguration(filePropertyHelper);
      
      stereoVisionBufferUpdaterRight.loadConfiguration(filePropertyHelper);
      mainUpdaterRight.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdaterRight.loadConfiguration(filePropertyHelper);
   }

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      boolean ocTreeUpdateSuccess = true;

      try
      {
         NormalOcTree mainOctreeLeft = mainUpdaterLeft.getMainOctree();
         NormalOcTree mainOctreeRight = mainUpdaterRight.getMainOctree();

         mainUpdaterLeft.clearOcTree();
         mainUpdaterRight.clearOcTree();
         mainUpdaterLeft.update();
         mainUpdaterRight.update();

         if (isThreadInterrupted())
            return;

         planarRegionFeatureUpdaterLeft.update(mainOctreeLeft);
         planarRegionFeatureUpdaterRight.update(mainOctreeRight);

         double distanceLeft = obstacleDistance(planarRegionFeatureUpdaterLeft.getPlanarRegionsList());
         double distanceRight = obstacleDistance(planarRegionFeatureUpdaterRight.getPlanarRegionsList());
         sender.sendDistance(distanceLeft > distanceRight ? "R: " + String.valueOf(distanceRight) : "L: " + String.valueOf(distanceLeft));

         planarRegionNetworkProviderLeft.update(ocTreeUpdateSuccess);
         planarRegionNetworkProviderRight.update(ocTreeUpdateSuccess);
         planarRegionNetworkProviderLeft.publishCurrentState();
         planarRegionNetworkProviderRight.publishCurrentState();
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
   }

   /*
    * returns -1 if there is no obstacle otherwise return distance to obstacle
    */
   private double obstacleDistance(PlanarRegionsList planarRegionsList) {      
      //params
      double distance = 999;            
      double positiveAngle = 135;
      double positiveD2Distance = 0.1;
      LinkedList<Double> distanceList = new LinkedList<Double>();
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         Vector3D vector = planarRegion.getNormal();
         //System.out.println(vector.toString());
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
         }               
      }
      
      //System.out.println("-----------------------------------");
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
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdaterLeft.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdaterRight.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
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

   public static ObstacleDisplayer createIntraprocessModule() throws Exception
   {
      String configurationFilePath = "./Configurations/defaultREAModuleConfiguration.txt";
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

      return new ObstacleDisplayer(messager, configurationFile);
   }
}