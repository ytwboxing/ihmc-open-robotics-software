package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.CharBuffer;
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
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
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

   protected static final boolean DEBUG = true;
   
   private final REAOcTreeBuffer stereoVisionBufferUpdaterLeft;
   private final REAOcTreeUpdater mainUpdaterLeft; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdaterLeft;
   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProviderLeft;   
   public static final double DEFAULT_DISTANCE_VALUE = 999.0;
   
   private final REAOcTreeBuffer stereoVisionBufferUpdaterRight;
   private final REAOcTreeUpdater mainUpdaterRight; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdaterRight;
   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProviderRight;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager1;  
   private final Messager reaMessager2;  
   
   private static UDPDataSender sender;
   private boolean nonEmptyLeftOcTree = false;
   private boolean nonEmptyRightOcTree = false;   
   
   //config parametrs
   private static int MAX_NUMBER_OF_POINTS = 20000;
   private static int THREAD_PERIOD_MILLISECONDS = 200;
   private static int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static double DEFAULT_OCTREE_RESOLUTION = 0.02;
   private static float MAXX = 0.0f;
   private static float MINX = -1.0f;
   private static float MAXY = 1.0f;
   private static float MINY = -0.1f;
   private static float MAXZ = 1.5f;
   private static float MINZ = 0.0f;   
   private static String PLANAR_REGIONS_SEGMENTATION_PARAMETERS = "search radius: 0.05, max distance from plane: 0.05, maxAngleFromPlane: 0.17453292519943295, minNormalQuality: 0.005, min region size: 50, max standard deviation: 0.015, min volumic density: 100000.0";
   private static double ANGLE_TO_CAMERA_TOLERANCE = 20.0;
   private static double ANGLE_BETWEEN_PLANES_TOLERANCE = 10.0;
   private static double XYZ_TOLERANCE = 0.1;
   private static double IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = 90.0;
   private static long TIME_BEFORE_NO_DISTANCE_REPORT = 2000;
   private static boolean PRINT_SENDER = false;
   private static double MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE = 0.05;
   private static double DISTANCE_CAMERA_FOOT= 0.635;
   private static double DISTANCE_FOOT_GROUND = 0.38; //0.38 for 4th and 5th dataset
   private static String ALGORITHM_SELECTOR = "stairDistance2"; //name of function   
   private static boolean DISTANCE_IN_FEET = false;

   private static final double distanceCameraGround = DISTANCE_CAMERA_FOOT + DISTANCE_FOOT_GROUND;
   
   private static final String MARK_MAX_NUMBER_OF_POINTS = "MAX_POINTS"; 
   private static final String MARK_THREAD_PERIOD_MILLISECONDS = "THREAD_PERIOD_MILLISECONDS";
   private static final String MARK_BUFFER_THREAD_PERIOD_MILLISECONDS = "BUFFER_THREAD_PERIOD_MILLISECONDS";   
   private static final String MARK_DEFAULT_OCTREE_RESOLUTION = "DEFAULT_OCTREE_RESOLUTION";
   private static final String MARK_MAXX = "MAXX";
   private static final String MARK_MINX = "MINX";
   private static final String MARK_MAXY = "MAXY";
   private static final String MARK_MINY = "MINY";
   private static final String MARK_MAXZ = "MAXZ";
   private static final String MARK_MINZ = "MINZ";
   private static final String MARK_PLANAR_REGIONS_SEGMENTATION_PARAMETERS = "PLANAR_REGIONS_SEGMENTATION_PARAMETERS";
   private static final String MARK_ANGLE_TO_CAMERA_TOLERANCE = "ANGLE_TO_CAMERA_TOLERANCE";
   private static final String MARK_ANGLE_BETWEEN_PLANES_TOLERANCE = "ANGLE_BETWEEN_PLANES_TOLERANCE";
   private static final String MARK_XYZ_TOLERANCE = "XYZ_TOLERANCE";
   private static final String MARK_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = "IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE";
   private static final String MARK_TIME_BEFORE_NO_DISTANCE_REPORT = "TIME_BEFORE_NO_DISTANCE_REPORT";
   private static final String MARK_PRINT_SENDER = "PRINT_SENDER";
   private static final String MARK_MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE = "MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE";
   private static final String MARK_DISTANCE_CAMERA_FOOT = "DISTANCE_CAMERA_FOOT";
   private static final String MARK_DISTANCE_FOOT_GROUND = "DISTANCE_FOOT_GROUND";
   private static final String MARK_ALGORITHM_SELECTOR = "ALGORITHM_SELECTOR";
   private static final String MARK_DISTANCE_IN_FEET = "DISTANCE_IN_FEET";
   
   //functions
   public static void main(String[] args)
   {
      try {
         loadConfFile();
         
         ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());
         
         //connect to cameras
         new RealSenseBridgeRos2("http://localhost:11311" // "http://192.168.137.2:11311" //   
                                 , "/cam_1/depth/color/points"
                                 , ros2Node
                                 , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Left"
                                 , MAX_NUMBER_OF_POINTS);   
         
         new RealSenseBridgeRos2("http://localhost:11311" // "http://192.168.137.2:11311" // 
                                 , "/cam_2/depth/color/points"
                                 , ros2Node
                                 , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Right"
                                 , MAX_NUMBER_OF_POINTS);       
         
         //main module
         ObstacleDisplayer module = ObstacleDisplayer.createIntraprocessModule();
         
         //sender to hololens
         sender = new UDPDataSender("192.168.0.13", 6669, PRINT_SENDER);
         
         module.start();  
         System.out.println("init complete");         
      }
      catch (Exception ex) {
         ex.printStackTrace();         
      }
   }

   private ObstacleDisplayer(Messager reaMessager1, Messager reaMessager2, File configurationFile) throws IOException
   {
      this.reaMessager1 = reaMessager1;
      this.reaMessager2 = reaMessager2;

      stereoVisionBufferUpdaterLeft = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager1, REAModuleAPI.StereoVisionBufferEnable, true,
                                                      REAModuleAPI.StereoVisionBufferOcTreeCapacity, 1000000, REAModuleAPI.StereoVisionBufferMessageCapacity, 1,
                                                      REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState);
      stereoVisionBufferUpdaterRight = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager2, REAModuleAPI.StereoVisionBufferEnable, true,
                                                          REAModuleAPI.StereoVisionBufferOcTreeCapacity, 1000000, REAModuleAPI.StereoVisionBufferMessageCapacity, 1,
                                                          REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState);
      REAOcTreeBuffer[] bufferUpdatersLeft = new REAOcTreeBuffer[] { stereoVisionBufferUpdaterLeft};
      REAOcTreeBuffer[] bufferUpdatersRight = new REAOcTreeBuffer[] { stereoVisionBufferUpdaterRight};
      mainUpdaterLeft = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdatersLeft, reaMessager1);
      mainUpdaterRight = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdatersRight, reaMessager2);
      planarRegionFeatureUpdaterLeft = new REAPlanarRegionFeatureUpdater(reaMessager1);
      planarRegionFeatureUpdaterRight = new REAPlanarRegionFeatureUpdater(reaMessager2);
      
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

      reaMessager1.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdaterLeft.saveConfiguration(filePropertyHelper));
      reaMessager1.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdaterLeft.saveConfiguration(filePropertyHelper));
      reaMessager1.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration, (content) -> planarRegionFeatureUpdaterLeft.saveConfiguration(filePropertyHelper));
      
      reaMessager2.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdaterRight.saveConfiguration(filePropertyHelper));
      reaMessager2.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdaterRight.saveConfiguration(filePropertyHelper));
      reaMessager2.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration, (content) -> planarRegionFeatureUpdaterRight.saveConfiguration(filePropertyHelper));
      
      planarRegionNetworkProviderLeft = new REAPlanarRegionPublicNetworkProvider(reaMessager1, planarRegionFeatureUpdaterLeft, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);
      planarRegionNetworkProviderRight = new REAPlanarRegionPublicNetworkProvider(reaMessager2, planarRegionFeatureUpdaterRight, ros2Node, publisherTopicNameGenerator,
                                                                                 subscriberTopicNameGenerator);
      
      reaMessager1.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      BoundingBoxParametersMessage boundingBox = new BoundingBoxParametersMessage();
      boundingBox.maxX = MAXX;
      boundingBox.minX = MINX;
      boundingBox.maxY = MAXY;
      boundingBox.minY = MINY; 
      boundingBox.maxZ = MAXZ;
      boundingBox.minZ = MINZ;
      reaMessager1.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBox);  

      reaMessager1.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager1.submitMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
      
      reaMessager2.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      boundingBox = new BoundingBoxParametersMessage();
      boundingBox.maxX = MAXX;
      boundingBox.minX = MINX;
      boundingBox.maxY = -MINY;//ON PURPOSE
      boundingBox.minY = -MAXY;
      boundingBox.maxZ = MAXZ;
      boundingBox.minZ = MINZ;
      reaMessager2.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBox); 

      reaMessager2.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager2.submitMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);

      reaMessager1.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, PlanarRegionSegmentationParameters.parse(PLANAR_REGIONS_SEGMENTATION_PARAMETERS));
      reaMessager2.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, PlanarRegionSegmentationParameters.parse(PLANAR_REGIONS_SEGMENTATION_PARAMETERS));      

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager1.submitMessage(REAModuleAPI.RequestEntireModuleState, true); 
      reaMessager2.submitMessage(REAModuleAPI.RequestEntireModuleState, true); 
   }
   
   private static void loadConfFile() throws Exception
   {
      File f = new File("OD.conf");
      if(f.exists()) {
         System.out.println("loading OD.conf");
         FileReader reader = new FileReader(f);
         BufferedReader bReader = new BufferedReader(reader);
         String line = bReader.readLine();
         while(line != null) {
            switch (line)
            {
               case MARK_MAX_NUMBER_OF_POINTS:
                  MAX_NUMBER_OF_POINTS = Integer.valueOf(bReader.readLine());
                  break;
               case MARK_THREAD_PERIOD_MILLISECONDS:
                  THREAD_PERIOD_MILLISECONDS = Integer.valueOf(bReader.readLine());                     
                  break;
               case MARK_BUFFER_THREAD_PERIOD_MILLISECONDS:
                  BUFFER_THREAD_PERIOD_MILLISECONDS = Integer.valueOf(bReader.readLine());  
                  break;
               case MARK_DEFAULT_OCTREE_RESOLUTION:
                  DEFAULT_OCTREE_RESOLUTION = Double.valueOf(bReader.readLine());                       
                  break;
               case MARK_MAXX:
                  MAXX = Float.valueOf(bReader.readLine());                   
                  break;
               case MARK_MINX:
                  MINX = Float.valueOf(bReader.readLine());                   
                  break;
               case MARK_MAXY:
                  MAXY = Float.valueOf(bReader.readLine());                   
                  break;
               case MARK_MINY:
                  MINY = Float.valueOf(bReader.readLine());                   
                  break;
               case MARK_MAXZ:
                  MAXZ = Float.valueOf(bReader.readLine());                   
                  break;
               case MARK_MINZ:
                  MINZ = Float.valueOf(bReader.readLine());                   
                  break;
               case MARK_PLANAR_REGIONS_SEGMENTATION_PARAMETERS:
                  PLANAR_REGIONS_SEGMENTATION_PARAMETERS = bReader.readLine();
                  break;
               case MARK_ANGLE_TO_CAMERA_TOLERANCE:
                  ANGLE_TO_CAMERA_TOLERANCE = Double.valueOf(bReader.readLine());                  
                  break; 
               case MARK_ANGLE_BETWEEN_PLANES_TOLERANCE:
                  ANGLE_BETWEEN_PLANES_TOLERANCE = Double.valueOf(bReader.readLine()); 
                  break;
               case MARK_XYZ_TOLERANCE:
                  XYZ_TOLERANCE = Double.valueOf(bReader.readLine()); 
                  break;
               case MARK_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE:
                  IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = Double.valueOf(bReader.readLine());                   
                  break;
               case MARK_TIME_BEFORE_NO_DISTANCE_REPORT:
                  TIME_BEFORE_NO_DISTANCE_REPORT = Long.valueOf(bReader.readLine());
                  break;
               case MARK_PRINT_SENDER:
                  PRINT_SENDER = Boolean.valueOf(bReader.readLine());
                  break;
               case MARK_MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE:
                  MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE = Double.valueOf(bReader.readLine());                   
                  break;
               case MARK_DISTANCE_CAMERA_FOOT:
                  DISTANCE_CAMERA_FOOT = Double.valueOf(bReader.readLine());                   
                  break;
               case MARK_DISTANCE_FOOT_GROUND:
                  DISTANCE_FOOT_GROUND = Double.valueOf(bReader.readLine()); 
                  break;
               case MARK_ALGORITHM_SELECTOR:
                  ALGORITHM_SELECTOR = bReader.readLine();
                  break;
               case MARK_DISTANCE_IN_FEET:
                  DISTANCE_IN_FEET = Boolean.valueOf(bReader.readLine());
                  break;
               default:
                  break;
            }
            
            line = bReader.readLine();
         } 
         bReader.close();          
      }      
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

   double distanceLeft = DEFAULT_DISTANCE_VALUE;
   double distanceRight = DEFAULT_DISTANCE_VALUE;
   int cycleCounter = 0;
   int runningMessageCounter = 0;
   long lastValidDistance = System.currentTimeMillis();
   private void mainUpdate()
   {      
      if (isThreadInterrupted())
         return;
      
      if(cycleCounter == 0) {
         cycleCounter = 100;
         System.out.println("running " + runningMessageCounter++);
      }
      cycleCounter--;

      boolean ocTreeUpdateSuccess = true;

      try
      {
         NormalOcTree mainOctreeLeft = mainUpdaterLeft.getMainOctree();
         NormalOcTree mainOctreeRight = mainUpdaterRight.getMainOctree();
         
         if(nonEmptyLeftOcTree == false && mainOctreeLeft.size() > 0) {
            nonEmptyLeftOcTree = true;
            System.out.println("left OcTree is not empty");
         }
         if(nonEmptyRightOcTree == false && mainOctreeRight.size() > 0) {
            nonEmptyRightOcTree = true;
            System.out.println("right OcTree is not empty");
         }

         mainUpdaterLeft.clearOcTree();
         mainUpdaterRight.clearOcTree();
         mainUpdaterLeft.update();
         mainUpdaterRight.update();

         if (isThreadInterrupted())
            return;

         planarRegionFeatureUpdaterLeft.update(mainOctreeLeft);
         planarRegionFeatureUpdaterRight.update(mainOctreeRight);

         //detection part
         switch(ALGORITHM_SELECTOR) {
            case "stairDistance2": 
               distanceLeft = stairDistance2(planarRegionFeatureUpdaterLeft.getPlanarRegionsList());
               distanceRight = stairDistance2(planarRegionFeatureUpdaterRight.getPlanarRegionsList());
               break;
            case "stairDistance": 
               distanceLeft = stairDistance(planarRegionFeatureUpdaterLeft.getPlanarRegionsList());
               distanceRight = stairDistance(planarRegionFeatureUpdaterRight.getPlanarRegionsList());
               break;
            case "obstacleDistance":  
               distanceLeft = obstacleDistance(planarRegionFeatureUpdaterLeft.getPlanarRegionsList());
               distanceRight = obstacleDistance(planarRegionFeatureUpdaterRight.getPlanarRegionsList());
               break;
            default:
               break;
         }

         if(distanceLeft != DEFAULT_DISTANCE_VALUE || distanceRight != DEFAULT_DISTANCE_VALUE) {
            lastValidDistance = System.currentTimeMillis();  
            
            if(distanceLeft > distanceRight) {
               if(DISTANCE_IN_FEET) {
                  sender.send("R: " + String.format("%.2f", (distanceRight/0.0328084)) + " feet");                      
               }
               else {
                  sender.send("R: " + String.format("%.2f", distanceRight) + " cm");                  
               }
            }
            else {
               if(DISTANCE_IN_FEET) {
                  sender.send("L: " + String.format("%.2f", (distanceLeft/0.0328084)) + " feet");                    
               }
               else {
                  sender.send("L: " + String.format("%.2f", distanceLeft) + " cm");
               }
            }
         }
         else if(System.currentTimeMillis() - lastValidDistance > TIME_BEFORE_NO_DISTANCE_REPORT){
            sender.send("free to go");
         }

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
    * returns distance to the closest stair, if no stair detected then returns DEFAULT_DISTANCE_VALUE
    * OBSOLETE
    */
   private double stairDistance(PlanarRegionsList planarRegionsList)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0)
         return DEFAULT_DISTANCE_VALUE;
      
      //variable
      double distance = DEFAULT_DISTANCE_VALUE;
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal();
         double angleToCamera = Math.acos(normalI.getZ())*180/Math.PI; //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - ANGLE_TO_CAMERA_TOLERANCE)
            continue;
        
         for(int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++) {
            if(j == i)
               continue;
            
            PlanarRegion planarRegionJ = planarRegionsList.getPlanarRegion(j);
            Vector3D normalJ = planarRegionJ.getNormal();
            double angleBetweenPlanes = normalJ.angle(normalI)*180/Math.PI;
            if(angleBetweenPlanes < 90.0 - ANGLE_BETWEEN_PLANES_TOLERANCE || angleBetweenPlanes > 90.0 + ANGLE_BETWEEN_PLANES_TOLERANCE)
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
            
            if(Math.abs(ImaxX - JminX) > XYZ_TOLERANCE) 
               continue;
            
            if(ImaxY < JminY || IminY > JmaxY)
               continue;            
            
            if(distance > JminZ)
               distance = JminZ;            
         }     
      }     
      
      return distance;  
   }
   
   /*
    * returns distance to the closest stair, if no stair detected then returns DEFAULT_DISTANCE_VALUE
    * different approach
    */
   private double stairDistance2(PlanarRegionsList planarRegionsList)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0)
         return DEFAULT_DISTANCE_VALUE;      
            
      //variable
      double distance = DEFAULT_DISTANCE_VALUE;

      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal();
         double angleToCamera = Math.acos(normalI.getZ())*180/Math.PI; //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - ANGLE_TO_CAMERA_TOLERANCE || angleToCamera > IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE + ANGLE_TO_CAMERA_TOLERANCE)
            continue;

         double minX = planarRegionI.getBoundingBox3dInWorld().getMinX();
         double minY = planarRegionI.getBoundingBox3dInWorld().getMinY();
         double minZ = planarRegionI.getBoundingBox3dInWorld().getMinZ();

         double maxX = planarRegionI.getBoundingBox3dInWorld().getMaxX();
         double maxY = planarRegionI.getBoundingBox3dInWorld().getMaxY();
         double maxZ = planarRegionI.getBoundingBox3dInWorld().getMaxZ();
         
         if(minZ > distanceCameraGround) {
            double expectedMinX = Math.sqrt(minZ*minZ - distanceCameraGround*distanceCameraGround) * -1.0;
            if(Math.abs(expectedMinX - minX) > MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE)
               continue;            
         }

         if(distance > minZ)
            distance = minZ; 
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
         if(angle > IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - ANGLE_TO_CAMERA_TOLERANCE) {
            double D2Distance = planarRegion.distanceToPointByProjectionOntoXYPlane(0.0, 0.0);
            if(D2Distance < XYZ_TOLERANCE) {
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
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdaterLeft.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdaterRight.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      reaMessager1.closeMessager();
      reaMessager2.closeMessager();
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
      String configurationFilePath = "/Configurations/defaultREAModuleConfiguration.txt";
      KryoMessager messager1 = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      KryoMessager messager2 = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager1.setAllowSelfSubmit(true);
      messager1.startMessager();
      messager2.setAllowSelfSubmit(true);
      messager2.startMessager();

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

      return new ObstacleDisplayer(messager1, messager2, configurationFile);
   }
}