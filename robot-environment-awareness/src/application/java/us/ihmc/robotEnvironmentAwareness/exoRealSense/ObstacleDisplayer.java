package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;
import java.util.concurrent.CopyOnWriteArrayList;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import std_msgs.msg.dds.Float64;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeBuffer;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeUpdater;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionFeatureUpdater;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

/*
 * main class that connect realsense D435 (RealSenseBridgeRos2), runs point cloud through planar region finder (most of this class), evaluate obstacles (function obstacleDistance) with the help of informations from Exo (callback subscribers on ros2Node) and present them to hololense (UDPDataSender) 
 */
public class ObstacleDisplayer
{   
   //variables
   private static Ros2Node ros2Node;
   private static UDPDataSender sender;
   
   private IHMCROS2Publisher<Float64> ExoLeftKneeHeightPublisher;
   private IHMCROS2Publisher<Float64> ExoRightKneeHeightPublisher;
   private IHMCROS2Publisher<Float64> ExoLeftThighAnglePublisher;
   private IHMCROS2Publisher<Float64> ExoRightThighAnglePublisher;   

   private BufferedWriter leftKneeHeightWriter;
   private BufferedWriter rightKneeHeightWriter;
   private BufferedWriter leftThighAngleWriter;
   private BufferedWriter rightThighAngleWriter; 
   
   protected static final boolean DEBUG = true;
   private final static double DEFAULT_VALUE = 500.0;  // 500 means leg in swing
   private final static double RIGHT_ANGLE_RAD = 1.5707963267948966192313216916398;
   private static boolean CREATE_DATASET = false;
   
   private final REAOcTreeBuffer stereoVisionBufferUpdaterLeft;
   private final REAOcTreeUpdater mainUpdaterLeft; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdaterLeft;    
   
   private final REAOcTreeBuffer stereoVisionBufferUpdaterRight;
   private final REAOcTreeUpdater mainUpdaterRight; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdaterRight;

   private final Messager reaMessager1;  
   private final Messager reaMessager2;  
   
   private boolean nonEmptyLeftOcTree = false;
   private boolean nonEmptyRightOcTree = false;   
   private boolean recievingLeftKneeHeight = false;
   private boolean recievingRightKneeHeight = false;
   private boolean recievingLeftThighAngle = false;
   private boolean recievingRightThighAngle = false;   
   
   //config parametrs
   private static int MAX_NUMBER_OF_POINTS = 300000;
   private static double DEFAULT_OCTREE_RESOLUTION = 0.02;
   private static float MAXX = 0.0f;
   private static float MINX = -1.0f;
   private static float MAXY = 1.0f;
   private static float MINY = -0.1f;
   private static float MAXZ = 2.0f;
   private static float MINZ = 0.0f;   
   private static String PLANAR_REGIONS_SEGMENTATION_PARAMETERS = "search radius: 0.05, max distance from plane: 0.05, maxAngleFromPlane: 0.17453292519943295, minNormalQuality: 0.005, min region size: 50, max standard deviation: 0.015, min volumic density: 100000.0";
   private static double ANGLE_CAMERA_PLANE_TOLERANCE = 10.0;
   private static double ANGLE_BETWEEN_PLANES_TOLERANCE = 10.0;
   private static double XYZ_TOLERANCE = 0.1;
   private static long TIME_BEFORE_NO_DISTANCE_REPORT = 2000;
   private static boolean PRINT_SENDER = false;
   private static double MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE = 0.05;   
   private static double DEFAULT_DISTANCE_CAMERA_GROUND = 0.625;
   private static double DEFAULT_THIGH_ANGLE = RIGHT_ANGLE_RAD;
   private static double DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = 90.0;
   private static String ALGORITHM_SELECTOR = "stairDistance3"; //name of function      
   private static boolean DISTANCE_IN_FEET = false;   
   private static int CAMERA_POSITION = 2; //1 - parallel with thight, 2 - pointing down 42°
   private static boolean PUBLISH_EXO = false;
   private static int DATASET_NUMBER = 1;
   private static double DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = 0.0;
   private static int CYCKE_COUNTER_THRESHOLD = 100;
   private static boolean PRINT_FINDER = false;
   private static boolean COPY_ON_WRITE_ARRAYS = true;
   
   private static double DISTANCE_LEFT_CAMERA_GROUND = DEFAULT_DISTANCE_CAMERA_GROUND;
   private static double DISTANCE_RIGHT_CAMERA_GROUND = DEFAULT_DISTANCE_CAMERA_GROUND;
   private static double LEFT_THIGH_ANGLE = DEFAULT_THIGH_ANGLE;
   private static double RIGHT_THIGH_ANGLE = DEFAULT_THIGH_ANGLE;
   private static double LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE;
   private static double RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE;
   private static double LEFT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE;
   private static double RIGHT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE;   
   
   private static final String MARK_MAX_NUMBER_OF_POINTS = "MAX_NUMBER_OF_POINTS"; 
   private static final String MARK_DEFAULT_OCTREE_RESOLUTION = "DEFAULT_OCTREE_RESOLUTION";
   private static final String MARK_MAXX = "MAXX";
   private static final String MARK_MINX = "MINX";
   private static final String MARK_MAXY = "MAXY";
   private static final String MARK_MINY = "MINY";
   private static final String MARK_MAXZ = "MAXZ";
   private static final String MARK_MINZ = "MINZ";   
   private static final String MARK_PLANAR_REGIONS_SEGMENTATION_PARAMETERS = "PLANAR_REGIONS_SEGMENTATION_PARAMETERS";
   private static final String MARK_ANGLE_CAMERA_PLANE_TOLERANCE = "ANGLE_CAMERA_PLANE_TOLERANCE";
   private static final String MARK_ANGLE_BETWEEN_PLANES_TOLERANCE = "ANGLE_BETWEEN_PLANES_TOLERANCE";
   private static final String MARK_XYZ_TOLERANCE = "XYZ_TOLERANCE";   
   private static final String MARK_TIME_BEFORE_NO_DISTANCE_REPORT = "TIME_BEFORE_NO_DISTANCE_REPORT";
   private static final String MARK_PRINT_SENDER = "PRINT_SENDER";
   private static final String MARK_MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE = "MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE";   
   private static final String MARK_DEFAULT_DISTANCE_CAMERA_GROUND = "DEFAULT_DISTANCE_CAMERA_GROUND";   
   private static final String MARK_DEFAULT_THIGH_ANGLE = "DEFAULT_THIGH_ANGLE";   
   private static final String MARK_DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = "DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE";   
   private static final String MARK_ALGORITHM_SELECTOR = "ALGORITHM_SELECTOR";   
   private static final String MARK_DISTANCE_IN_FEET = "DISTANCE_IN_FEET";   
   private static final String MARK_CAMERA_POSITION = "CAMERA_POSITION"; 
   private static final String MARK_PUBLISH_EXO = "PUBLISH_EXO";
   private static final String MARK_DATASET_NUMBER = "DATASET_NUMBER";
   private static final String MARK_DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = "DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE";
   private static final String MARK_CYCKE_COUNTER_THRESHOLD = "CYCKE_COUNTER_THRESHOLD";
   private static final String MARK_PRINT_FINDER = "PRINT_FINDER";
   private static final String MARK_COPY_ON_WRITE_ARRAYS = "COPY_ON_WRITE_ARRAYS";
   
   //functions
   public static void main(String[] args)
   {
      try {
         loadConfFile();
         
         ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());

         //connect to cameras
         new RealSenseBridgeRos2("http://10.7.4.52:11311"  // "http://localhost:11311"
                                 , "/cam_1/depth/color/points"
                                 , ros2Node
                                 , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Left"
                                 , MAX_NUMBER_OF_POINTS);   
         
         new RealSenseBridgeRos2("http://10.7.4.52:11311"  //"http://localhost:11311" 
                                 , "/cam_2/depth/color/points"
                                 , ros2Node
                                 , ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(StereoVisionPointCloudMessage.class) + "Right"
                                 , MAX_NUMBER_OF_POINTS); 
         
         //main module
         ObstacleDisplayer module = ObstacleDisplayer.createIntraprocessModule();
         
         //sender to hololens
         //todo JOBY uncomment
         //sender = new UDPDataSender("192.168.0.13", 6669, PRINT_SENDER);
         
         System.out.println("init complete");
         Scanner commandScanner = new Scanner(System.in);
         while (true)
         {
            String command = commandScanner.next();

            if (command.contains("s"))
            {               
               CREATE_DATASET = true;
               System.out.println("s pressed");
            }
            else if (command.contains("d"))
            {
               CREATE_DATASET = false;
               System.out.println("d pressed");
            }
         }
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
      
      ROS2Tools.createCallbackSubscription(ros2Node
                                           , std_msgs.msg.dds.String.class
                                           , "mina_v2/knee_height/left"
                                           , this::handleExoLeftKneeHeight);
      ROS2Tools.createCallbackSubscription(ros2Node
                                           , std_msgs.msg.dds.String.class
                                           , "mina_v2/knee_height/right"
                                           , this::handleExoRightKneeHeight);
      ROS2Tools.createCallbackSubscription(ros2Node
                                           , std_msgs.msg.dds.String.class
                                           , "mina_v2/thigh_angle/left"
                                           , this::handleExoLeftThighAngle);
      ROS2Tools.createCallbackSubscription(ros2Node
                                           , std_msgs.msg.dds.String.class
                                           , "mina_v2/thigh_angle/right"
                                           , this::handleExoRightThighAngle);

      if(PUBLISH_EXO) {
         ExoLeftKneeHeightPublisher = ROS2Tools.createPublisher(ros2Node, Float64.class, "knee_height/left");
         ExoRightKneeHeightPublisher = ROS2Tools.createPublisher(ros2Node, Float64.class, "knee_height/right");
         ExoLeftThighAnglePublisher = ROS2Tools.createPublisher(ros2Node, Float64.class, "thigh_angle/left");
         ExoRightThighAnglePublisher = ROS2Tools.createPublisher(ros2Node, Float64.class, "thigh_angle/right");         
      }      

      leftKneeHeightWriter = new BufferedWriter(new FileWriter("DATASETS/" + DATASET_NUMBER + "/EXO/LKneeHeight.txt")); 
      rightKneeHeightWriter = new BufferedWriter(new FileWriter("DATASETS/" + DATASET_NUMBER + "/EXO/RKneeHeight.txt"));  
      leftThighAngleWriter = new BufferedWriter(new FileWriter("DATASETS/" + DATASET_NUMBER + "/EXO/LThighAngle.txt"));   
      rightThighAngleWriter = new BufferedWriter(new FileWriter("DATASETS/" + DATASET_NUMBER + "/EXO/RThighAngle.txt"));   

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

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

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      stereoVisionBufferUpdaterLeft.loadConfiguration(filePropertyHelper);
      mainUpdaterLeft.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdaterLeft.loadConfiguration(filePropertyHelper);
      
      stereoVisionBufferUpdaterRight.loadConfiguration(filePropertyHelper);
      mainUpdaterRight.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdaterRight.loadConfiguration(filePropertyHelper);
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
               case MARK_ANGLE_CAMERA_PLANE_TOLERANCE:
                  ANGLE_CAMERA_PLANE_TOLERANCE = Double.valueOf(bReader.readLine());                  
                  break;                   
               case MARK_ANGLE_BETWEEN_PLANES_TOLERANCE:
                  ANGLE_BETWEEN_PLANES_TOLERANCE = Double.valueOf(bReader.readLine()); 
                  break;                  
               case MARK_XYZ_TOLERANCE:
                  XYZ_TOLERANCE = Double.valueOf(bReader.readLine()); 
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
               case MARK_DEFAULT_DISTANCE_CAMERA_GROUND:
                  DEFAULT_DISTANCE_CAMERA_GROUND = Double.valueOf(bReader.readLine());                   
                  break;                  
               case MARK_DEFAULT_THIGH_ANGLE:
                  DEFAULT_THIGH_ANGLE = Double.valueOf(bReader.readLine());
                  break;
               case MARK_DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE:
                  DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = Double.valueOf(bReader.readLine());
                  break;                  
               case MARK_ALGORITHM_SELECTOR:
                  ALGORITHM_SELECTOR = bReader.readLine();
                  break;
               case MARK_DISTANCE_IN_FEET:
                  DISTANCE_IN_FEET = Boolean.valueOf(bReader.readLine());
                  break;
               case MARK_CAMERA_POSITION:
                  CAMERA_POSITION = Integer.valueOf(bReader.readLine());
                  break;
               case MARK_PUBLISH_EXO:
                  PUBLISH_EXO = Boolean.valueOf(bReader.readLine());
                  break;
               case MARK_DATASET_NUMBER:
                  DATASET_NUMBER = Integer.valueOf(bReader.readLine());
                  break;
               case MARK_DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE:
                  DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = Double.valueOf(bReader.readLine());
                  break;
               case MARK_CYCKE_COUNTER_THRESHOLD:
                  CYCKE_COUNTER_THRESHOLD = Integer.valueOf(bReader.readLine());
                  break;
               case MARK_PRINT_FINDER:
                  PRINT_FINDER = Boolean.valueOf(bReader.readLine());
                  break;
               case MARK_COPY_ON_WRITE_ARRAYS:
                  COPY_ON_WRITE_ARRAYS = Boolean.valueOf(bReader.readLine());
                  break;
               default:
                  break;
            }
            
            line = bReader.readLine();
         } 
         bReader.close(); 
         
         DISTANCE_LEFT_CAMERA_GROUND = DEFAULT_DISTANCE_CAMERA_GROUND;
         DISTANCE_RIGHT_CAMERA_GROUND = DEFAULT_DISTANCE_CAMERA_GROUND;
         LEFT_THIGH_ANGLE = DEFAULT_THIGH_ANGLE;
         RIGHT_THIGH_ANGLE = DEFAULT_THIGH_ANGLE;
         LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE;
         RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE;
         LEFT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE;
         RIGHT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = DEFAULT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE; 
      }      
   }

   NormalOcTree bufferOctreeLeft = new NormalOcTree(DEFAULT_OCTREE_RESOLUTION);
   private void dispatchStereoVisionPointCloudMessageLeft(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      stereoVisionBufferUpdaterLeft.handleStereoVisionPointCloudMessage(message);      
      mainUpdaterLeft.handleStereoVisionPointCloudMessage(message);
      
      stereoVisionBufferUpdaterLeft.runMethod(bufferOctreeLeft);      
      mainUpdateLeft(message.getTimestamp(), message);
   }

   NormalOcTree bufferOctreeRight = new NormalOcTree(DEFAULT_OCTREE_RESOLUTION);
   private void dispatchStereoVisionPointCloudMessageRight(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      stereoVisionBufferUpdaterRight.handleStereoVisionPointCloudMessage(message);      
      mainUpdaterRight.handleStereoVisionPointCloudMessage(message);
      
      stereoVisionBufferUpdaterRight.runMethod(bufferOctreeRight);
      mainUpdateRight(message.getTimestamp(), message);
   }

   List<Float64> exoLeftKneeHeightList = COPY_ON_WRITE_ARRAYS ? new ArrayList<Float64>() : new CopyOnWriteArrayList<Float64>();     
   private void handleExoLeftKneeHeight(Subscriber<std_msgs.msg.dds.String> subscriber) {
      std_msgs.msg.dds.String s = subscriber.takeNextData();
      String[] values = s.data_.toString().split(";");
      Float64 f = new Float64();
      f.setUniqueId(Long.valueOf(values[0]));
      f.setData(Double.valueOf(values[1]));      
      
      exoLeftKneeHeightList.add(f);   

      if(PUBLISH_EXO)
         ExoLeftKneeHeightPublisher.publish(new Float64(f));
      
      if(recievingLeftKneeHeight == false) {
         recievingLeftKneeHeight = true;
         System.out.println("recieving Left Knee Height");
      }
   } 

   List<Float64> exoRightKneeHeightList = COPY_ON_WRITE_ARRAYS ? new ArrayList<Float64>() : new CopyOnWriteArrayList<Float64>();   
   private void handleExoRightKneeHeight(Subscriber<std_msgs.msg.dds.String> subscriber) {
      std_msgs.msg.dds.String s = subscriber.takeNextData();
      String[] values = s.data_.toString().split(";");
      Float64 f = new Float64();
      f.setUniqueId(Long.valueOf(values[0]));
      f.setData(Double.valueOf(values[1])); 
      
      exoRightKneeHeightList.add(f);       
      
      if(PUBLISH_EXO)
         ExoRightKneeHeightPublisher.publish(new Float64(f));

      if(recievingRightKneeHeight == false) {
         recievingRightKneeHeight = true;
         System.out.println("recieving Right Knee Height");
      }      
   }

   List<Float64> exoLeftThighAngleList = COPY_ON_WRITE_ARRAYS ? new ArrayList<Float64>() : new CopyOnWriteArrayList<Float64>();      
   private void handleExoLeftThighAngle(Subscriber<std_msgs.msg.dds.String> subscriber) {
      std_msgs.msg.dds.String s = subscriber.takeNextData();
      String[] values = s.data_.toString().split(";");
      Float64 f = new Float64();
      f.setUniqueId(Long.valueOf(values[0]));
      f.setData(Double.valueOf(values[1])); 
      
      exoLeftThighAngleList.add(f);       
      
      if(PUBLISH_EXO)
         ExoLeftThighAnglePublisher.publish(new Float64(f));

      if(recievingLeftThighAngle == false) {
         recievingLeftThighAngle = true;
         System.out.println("recieving Left Thigh Angle");
      }      
   }

   List<Float64> exoRightThighAngleList = COPY_ON_WRITE_ARRAYS ? new ArrayList<Float64>() : new CopyOnWriteArrayList<Float64>();   
   private void handleExoRightThighAngle(Subscriber<std_msgs.msg.dds.String> subscriber) {
      std_msgs.msg.dds.String s = subscriber.takeNextData();
      String[] values = s.getData().toString().split(";");
      Float64 f = new Float64();
      f.setUniqueId(Long.valueOf(values[0]));
      f.setData(Double.valueOf(values[1])); 
      
      exoRightThighAngleList.add(f);      
      
      if(PUBLISH_EXO)
         ExoRightThighAnglePublisher.publish(new Float64(f));
      
      if(recievingRightThighAngle == false) {
         recievingRightThighAngle = true;
         System.out.println("recieving Right Thigh Angle");
      }      
   }
   
   double distanceLeft = DEFAULT_VALUE;
   int cycleCounterLeft = 0;
   int runningMessageCounterLeft = 0;
   int savingIndexLeft = 0;
   private void mainUpdateLeft(Long pointCloudTimeStamp, StereoVisionPointCloudMessage message)
   {
      //time from Up^2 is in nanoseconds -> need to get them in milliseconds
      pointCloudTimeStamp = Math.round(pointCloudTimeStamp/1000000.0);  
      
      //running messages 
      if(cycleCounterLeft == CYCKE_COUNTER_THRESHOLD) {
         cycleCounterLeft = 0;
         System.out.println("running left " + runningMessageCounterLeft++);
      }
      else {
         cycleCounterLeft++;         
      }

      try
      {
         //planar region stuff
         NormalOcTree mainOctreeLeft = mainUpdaterLeft.getMainOctree();
         mainUpdaterLeft.clearOcTree();
         mainUpdaterLeft.update();
         planarRegionFeatureUpdaterLeft.update(mainOctreeLeft);
         if(nonEmptyLeftOcTree == false && mainOctreeLeft.size() > 0) {
            nonEmptyLeftOcTree = true;
            System.out.println("left OcTree is not empty");
         }

         //exo data calculation
         double kneeHeight = findClosestAndRemoverUnnecessary(exoLeftKneeHeightList, pointCloudTimeStamp);
         double tightAngle = findClosestAndRemoverUnnecessary(exoLeftThighAngleList, pointCloudTimeStamp); 

         if(tightAngle != DEFAULT_VALUE){
            LEFT_THIGH_ANGLE = tightAngle;  
            
            switch (CAMERA_POSITION)
            {
               case 1:
                  LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = (180/Math.PI)* (RIGHT_ANGLE_RAD - (LEFT_THIGH_ANGLE - RIGHT_ANGLE_RAD)); // 1.5708 rad (90°) is ideal angle between camera view vector(forward) and stair 
                  LEFT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - 90.0;
                  break;
               case 2:
                  LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = (180/Math.PI)* ((RIGHT_ANGLE_RAD + 0.698132) - (LEFT_THIGH_ANGLE - RIGHT_ANGLE_RAD)); // when camera is looking down, where is additional 0.698132 rad (40°)          
                  LEFT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - 90.0;
                  break;
               default:
                  break;
            }
         }         
         
         if(kneeHeight == DEFAULT_VALUE) {
            DISTANCE_LEFT_CAMERA_GROUND = DEFAULT_VALUE;
         }
         else {
            switch (CAMERA_POSITION)
            {
               case 1:
                  DISTANCE_LEFT_CAMERA_GROUND = kneeHeight + Math.sin(LEFT_THIGH_ANGLE - 0.69001592) * 0.0943; //0.69001592 rad (39.535°) and 0.0943 meters are measurements from exo
                  break;
               case 2:
                  DISTANCE_LEFT_CAMERA_GROUND = kneeHeight + Math.sin(LEFT_THIGH_ANGLE - 0.265586752) * 0.0631; //0.265586752 rad (15.217°) and 0.0631 meters are measurements from exo
                  break;
               default:
                  break;
            }
         }
         
         //saving
         if(CREATE_DATASET) {
            try
            {
               leftThighAngleWriter.write(String.valueOf(pointCloudTimeStamp) + "\n" + String.valueOf(tightAngle) + "\n");  
               leftThighAngleWriter.flush();
            }
            catch (Exception ex)
            {
               CREATE_DATASET = false;
               ex.printStackTrace();
            } 
            
            try
            {
               leftKneeHeightWriter.write(String.valueOf(pointCloudTimeStamp) + "\n" + String.valueOf(kneeHeight) + "\n");    
               leftKneeHeightWriter.flush();
            }
            catch (Exception ex)
            {
               CREATE_DATASET = false;
               ex.printStackTrace();
            }  
            
            try
            {
               us.ihmc.idl.IDLSequence.Float cloud = message.getPointCloud();
               float[] pointCloud = cloud.toArray();
               
               File file = new File("DATASETS/" + DATASET_NUMBER + "/LPC/stereovision_pointcloud_" + savingIndexLeft + ".txt");
               FileWriter fileWriter = new FileWriter(file);
               StringBuilder builder = new StringBuilder("");
               for (int i = 0; i < pointCloud.length/3; i++)
               {
                  builder.append(i + "\t" + pointCloud[i*3] + "\t" + pointCloud[i*3+1] + "\t" + pointCloud[i*3+2] + "\t-16777216\n");
               }
               fileWriter.write(builder.toString());
               
               fileWriter.close();
               savingIndexLeft++;
            }
            catch (Exception ex)
            {
               CREATE_DATASET = false;
               ex.printStackTrace();
            }
         }

         //detection part
         switch(ALGORITHM_SELECTOR) {
            case "stairDistance3": 
               distanceLeft = stairDistance3(planarRegionFeatureUpdaterLeft.getPlanarRegionsList(), DISTANCE_LEFT_CAMERA_GROUND, LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE, LEFT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE);
               break;
            case "stairDistance2": 
               distanceLeft = stairDistance2(planarRegionFeatureUpdaterLeft.getPlanarRegionsList(), DISTANCE_LEFT_CAMERA_GROUND, LEFT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE, LEFT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE);
               break;
            case "stairDistance": 
               distanceLeft = stairDistance(planarRegionFeatureUpdaterLeft.getPlanarRegionsList());
               break;
            case "obstacleDistance":  
               distanceLeft = obstacleDistance(planarRegionFeatureUpdaterLeft.getPlanarRegionsList());
               break;
            default:
               break;
         }
         
         evaluateAndSend();
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }
   }

   double distanceRight = DEFAULT_VALUE;
   int cycleCounterRight = 0;
   int runningMessageCounterRight = 0;  
   int savingIndexRight = 0;
   private void mainUpdateRight(Long pointCloudTimeStamp, StereoVisionPointCloudMessage message)
   {  
      //time from Up^2 is in nanoseconds -> need to get them in milliseconds
      pointCloudTimeStamp = Math.round(pointCloudTimeStamp/1000000.0);      
      
      //running messages      
      if(cycleCounterRight == CYCKE_COUNTER_THRESHOLD) {
         cycleCounterRight = 0;
         System.out.println("running right :" + runningMessageCounterRight++);
      }
      else {
         cycleCounterRight++;         
      }

      try
      {
         //planar region stuff
         NormalOcTree mainOctreeRight = mainUpdaterRight.getMainOctree();
         mainUpdaterRight.clearOcTree();
         mainUpdaterRight.update();
         planarRegionFeatureUpdaterRight.update(mainOctreeRight);         
         if(nonEmptyRightOcTree == false && mainOctreeRight.size() > 0) {
            nonEmptyRightOcTree = true;
            System.out.println("right OcTree is not empty");
         }
         
         //exo data calculation
         double kneeHeight = findClosestAndRemoverUnnecessary(exoRightKneeHeightList, pointCloudTimeStamp);
         double tightAngle = findClosestAndRemoverUnnecessary(exoRightThighAngleList, pointCloudTimeStamp);
         
         if(tightAngle != DEFAULT_VALUE) {            
            RIGHT_THIGH_ANGLE = tightAngle; 
            
            switch (CAMERA_POSITION)
            {
               case 1:
                  RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = (180/Math.PI)* (RIGHT_ANGLE_RAD - (RIGHT_THIGH_ANGLE - RIGHT_ANGLE_RAD)); // 1.5708 rad (90°) is ideal angle between camera view vector(forward) and step 
                  RIGHT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - 90.0;
                  break;
               case 2:
                  RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE = (180/Math.PI)* ((RIGHT_ANGLE_RAD + 0.698132) - (LEFT_THIGH_ANGLE - RIGHT_ANGLE_RAD)); // when camera is looking down, where is additional 0.698132 rad (40°)
                  RIGHT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE = RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - 90.0;
                  break;
               default:
                  break;
            }            
         }
         
         if(kneeHeight == DEFAULT_VALUE) {
            DISTANCE_RIGHT_CAMERA_GROUND = DEFAULT_VALUE;
         }      
         else {
            switch (CAMERA_POSITION)
            {
               case 1:
                  DISTANCE_RIGHT_CAMERA_GROUND = kneeHeight + Math.sin(RIGHT_THIGH_ANGLE - 0.69001592) * 0.0943; //0.69001592 rad (39.535°) and 0.0943 meters are measurements from exo
                  break;
               case 2:
                  DISTANCE_RIGHT_CAMERA_GROUND = kneeHeight + Math.sin(RIGHT_THIGH_ANGLE - 0.265586752) * 0.0631; //0.265586752 rad (15.217°) and 0.0631 meters are measurements from exo
                  break;
               default:
                  break;
            }            
         }
         
         //saving
         if(CREATE_DATASET) {
            try
            {
               rightThighAngleWriter.write(String.valueOf(pointCloudTimeStamp) + "\n" + String.valueOf(tightAngle) + "\n");
               rightThighAngleWriter.flush();
            }
            catch (Exception ex)
            {
               CREATE_DATASET = false;
               ex.printStackTrace();
            } 

            try
            {
               rightKneeHeightWriter.write(String.valueOf(pointCloudTimeStamp) + "\n" + String.valueOf(kneeHeight) + "\n");   
               rightKneeHeightWriter.flush();
            }
            catch (Exception ex)
            {
               CREATE_DATASET = false;
               ex.printStackTrace();
            } 
            
            try
            {
               us.ihmc.idl.IDLSequence.Float cloud = message.getPointCloud();
               float[] pointCloud = cloud.toArray();
               
               File file = new File("DATASETS/" + DATASET_NUMBER + "/RPC/stereovision_pointcloud_" + savingIndexRight + ".txt");
               FileWriter fileWriter = new FileWriter(file);
               StringBuilder builder = new StringBuilder("");
               for (int i = 0; i < pointCloud.length/3; i++)
               {
                  builder.append(i + "\t" + pointCloud[i*3] + "\t" + pointCloud[i*3+1] + "\t" + pointCloud[i*3+2] + "\t-16777216\n");
               }
               fileWriter.write(builder.toString());
               
               fileWriter.close();
               savingIndexRight++;
            }
            catch (Exception ex)
            {
               CREATE_DATASET = false;
               ex.printStackTrace();
            }
         }

         //detection part
         switch(ALGORITHM_SELECTOR) {
            case "stairDistance3": 
               distanceRight = stairDistance3(planarRegionFeatureUpdaterRight.getPlanarRegionsList(), DISTANCE_RIGHT_CAMERA_GROUND, RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE, RIGHT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE);
               break;
            case "stairDistance2": 
               distanceRight = stairDistance2(planarRegionFeatureUpdaterRight.getPlanarRegionsList(), DISTANCE_RIGHT_CAMERA_GROUND, RIGHT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE, RIGHT_IDEAL_ANGLE_BETWEEN_GROUND_AND_PLANE);
               break;
            case "stairDistance": 
               distanceRight = stairDistance(planarRegionFeatureUpdaterRight.getPlanarRegionsList());
               break;
            case "obstacleDistance":  
               distanceRight = obstacleDistance(planarRegionFeatureUpdaterRight.getPlanarRegionsList());
               break;
            default:
               break;
         }
         
         //todo JOBY uncomment
         //evaluateAndSend();
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }
   }

   /*
    * find closes Float64.uniqueId (timestamp) to pointCloudTimeStamp and return Float64.data_
    */
   private double findClosestAndRemoverUnnecessary(List<Float64> list, Long pointCloudTimeStamp)
   {
      if(list == null || list.size() == 0)
         return DEFAULT_VALUE;
      
      int index = (int)(list.size()/2.0f);
      int indexMover = index/2;
      if(indexMover == 0)
         indexMover++;
      
      while(true) {
         Float64 f = list.get(index);
         if(pointCloudTimeStamp < f.getUniqueId()) {
            if(index == 0) {
               if(PRINT_FINDER)
                  System.out.println("i: " + index + ", size: " + list.size() + " difference: " + Math.abs(pointCloudTimeStamp - f.getUniqueId()));               
               return f.getData();
            }
            if(list.get(index-1).getUniqueId() < pointCloudTimeStamp) {
               if(Math.abs(list.get(index-1).getUniqueId() - pointCloudTimeStamp) > Math.abs(f.getUniqueId() - pointCloudTimeStamp)) {
                  if(PRINT_FINDER)
                     System.out.println("i: " + index + ", size: " + list.size() + " difference: " + Math.abs(pointCloudTimeStamp - f.getUniqueId()));
                  oldDataRemove(list, index-1);
                  return f.getData();
               }
               else { 
                  if(PRINT_FINDER)
                     System.out.println("i: " + (index-1) + ", size: " + list.size() + " difference: " + Math.abs(pointCloudTimeStamp - list.get(index-1).getUniqueId()));
                  f = list.get(index-1);
                  oldDataRemove(list, index-2);
                  return f.getData();
               }
            }
            
            index -= indexMover;
         }
         else {
            if(index == list.size() - 1) {
               if(PRINT_FINDER)
                  System.out.println("i: " + index + ", size: " + list.size() + " difference: " + Math.abs(pointCloudTimeStamp - f.getUniqueId()));
               list.clear();
               list.add(f);
               return f.getData();               
            }
            if(pointCloudTimeStamp < list.get(index+1).getUniqueId()) {
               if(Math.abs(list.get(index+1).getUniqueId() - pointCloudTimeStamp) > Math.abs(f.getUniqueId() - pointCloudTimeStamp)) {
                  if(PRINT_FINDER)
                     System.out.println("i: " + index + ", size: " + list.size() + " difference: " + Math.abs(pointCloudTimeStamp - f.getUniqueId()));
                  oldDataRemove(list, index-1);
                  return f.getData();
               }
               else {
                  if(PRINT_FINDER)
                     System.out.println("i: " + (index+1) + ", size: " + list.size() + " difference: " + Math.abs(pointCloudTimeStamp - list.get(index+1).getUniqueId()));
                  f = list.get(index+1);
                  oldDataRemove(list, index);
                  return f.getData();
               }
            }
            
            index += indexMover;          
         }
         indexMover /= 2;
         if(indexMover == 0)
            indexMover++;
      }
   }
   
   private void oldDataRemove(List<Float64> list, int index)
   {
      while(index >= 0) {
         list.remove(0);         
         index--;
      }
   }
   

   long lastValidDistance = System.currentTimeMillis();
   private void evaluateAndSend()
   {
      if(distanceLeft != DEFAULT_VALUE || distanceRight != DEFAULT_VALUE) {
         lastValidDistance = System.currentTimeMillis();  
         
         if(distanceLeft > distanceRight) {
            if(DISTANCE_IN_FEET) {
               sender.send("R: " + String.format("%.2f", (distanceRight*3.28084)) + " feet");                      
            }
            else {
               sender.send("R: " + String.format("%.2f", distanceRight) + " meters");                  
            }
         }
         else {
            if(DISTANCE_IN_FEET) {
               sender.send("L: " + String.format("%.2f", (distanceLeft*3.28084)) + " feet");                    
            }
            else {
               sender.send("L: " + String.format("%.2f", distanceLeft) + " meters");
            }
         }
      }
      else if(System.currentTimeMillis() - lastValidDistance > TIME_BEFORE_NO_DISTANCE_REPORT){
         sender.send("free to go");
      }      
   }
   
   /*
    * returns distance to the closest stair, if no stair detected then returns DEFAULT_DISTANCE_VALUE
    * OBSOLETE
    */
   private double stairDistance(PlanarRegionsList planarRegionsList)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0)
         return DEFAULT_VALUE;
      
      //variable
      double distance = DEFAULT_VALUE;
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal();
         double angleToCamera = Math.acos(normalI.getZ())*(180/Math.PI); //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < DEFAULT_IDEAL_ANGLE_BETWEEN_CAMERA_AND_PLANE - ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;
        
         for(int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++) {
            if(j == i)
               continue;
            
            PlanarRegion planarRegionJ = planarRegionsList.getPlanarRegion(j);
            Vector3D normalJ = planarRegionJ.getNormal();
            double angleBetweenPlanes = normalJ.angle(normalI)*(180/Math.PI);
            if(angleBetweenPlanes < 90.0 - ANGLE_BETWEEN_PLANES_TOLERANCE || angleBetweenPlanes > 90.0 + ANGLE_BETWEEN_PLANES_TOLERANCE)
               continue;           

            double IminY = planarRegionI.getBoundingBox3dInWorld().getMinY();

            double ImaxX = planarRegionI.getBoundingBox3dInWorld().getMaxX();
            double ImaxY = planarRegionI.getBoundingBox3dInWorld().getMaxY();           

            double JminX = planarRegionJ.getBoundingBox3dInWorld().getMinX();
            double JminY = planarRegionJ.getBoundingBox3dInWorld().getMinY();
            double JminZ = planarRegionJ.getBoundingBox3dInWorld().getMinZ();
            
            double JmaxY = planarRegionJ.getBoundingBox3dInWorld().getMaxY();
            
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
    * OBSOLETE
    */
   private double stairDistance2(PlanarRegionsList planarRegionsList, double distanceCameraGround, double idealAngleBetweenCameraAndPlane, double idealAngleBetweenGroundAndPlane)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0 || distanceCameraGround == DEFAULT_VALUE)
         return DEFAULT_VALUE;      
            
      //variable
      double distance = DEFAULT_VALUE;

      RotationScaleMatrix rotation = null;
      Point3D translation = null;
      AffineTransform transform = null; 
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal();
         double angleToCamera = Math.acos(normalI.getZ())*(180/Math.PI); //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < idealAngleBetweenCameraAndPlane - ANGLE_CAMERA_PLANE_TOLERANCE || angleToCamera > idealAngleBetweenCameraAndPlane + ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;

         double angleToGround = Math.acos(normalI.getX())*(180/Math.PI); //simplified for ground vector (1, 0, 0)
         if(angleToGround < Math.abs(idealAngleBetweenGroundAndPlane) - ANGLE_CAMERA_PLANE_TOLERANCE || angleToGround > Math.abs(idealAngleBetweenGroundAndPlane) + ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;
         
         if(rotation == null) {
            rotation = new RotationScaleMatrix();
            rotation.setEuler(0.0, -idealAngleBetweenGroundAndPlane*(Math.PI/180), 0.0);
            translation = new Point3D(0.0, 0.0, 0.0);
            transform = new AffineTransform(rotation, translation); 
         }
         
         Point3D minPoint = (Point3D) planarRegionI.getBoundingBox3dInWorld().getMinPoint();
         if(planarRegionI.getNormal().getZ() > 0) 
            minPoint.setX(planarRegionI.getBoundingBox3dInWorld().getMaxPoint().getX());
         minPoint.applyTransform(transform);
         
         double pointToGround = minPoint.getX() * -1.0; 
         if(pointToGround > distanceCameraGround - MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE && pointToGround < distanceCameraGround + MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE)
            continue;
         
         if(distance > minPoint.getZ())
            distance =  minPoint.getZ();
         
         double stairHeight = distanceCameraGround - pointToGround;
      }
      
      return distance;
   }   

   /*
    * returns distance to the closest stair, if no stair detected then returns DEFAULT_DISTANCE_VALUE
    * also different approach
    */
   private double stairDistance3(PlanarRegionsList planarRegionsList, double distanceCameraGround, double idealAngleBetweenCameraAndPlane, double idealAngleBetweenGroundAndPlane)
   {  
      if(planarRegionsList.getNumberOfPlanarRegions() == 0 || distanceCameraGround == DEFAULT_VALUE) 
         return DEFAULT_VALUE;      
            
      //variable
      double distance = DEFAULT_VALUE;           
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegionI = planarRegionsList.getPlanarRegion(i);
         Vector3D normalI = planarRegionI.getNormal(); 
         Point3D max = (Point3D) planarRegionI.getBoundingBox3dInWorld().getMaxPoint();
         Point3D min = (Point3D) planarRegionI.getBoundingBox3dInWorld().getMinPoint();
         
         //to consider planes to be stair of floor, they must have particular angels
         double lenghtOfSimplifiedVector = Math.sqrt(normalI.getX()*normalI.getX() + normalI.getZ()*normalI.getZ()); //ignoring Y axis
         double angleToCamera = Math.acos(normalI.getZ() / lenghtOfSimplifiedVector) *(180/Math.PI); //simplified for cemara vector (0, 0, 1)
         if(angleToCamera < idealAngleBetweenCameraAndPlane - ANGLE_CAMERA_PLANE_TOLERANCE || angleToCamera > idealAngleBetweenCameraAndPlane + ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;
         double angleToGround = Math.acos(normalI.getX() / lenghtOfSimplifiedVector) *(180/Math.PI); //simplified for ground vector (1, 0, 0)
         if(angleToGround < Math.abs(idealAngleBetweenGroundAndPlane) - ANGLE_CAMERA_PLANE_TOLERANCE || angleToGround > Math.abs(idealAngleBetweenGroundAndPlane) + ANGLE_CAMERA_PLANE_TOLERANCE)
            continue;

         //rotating points before distinguishing between stair and floor
         RotationScaleMatrix rotation = new RotationScaleMatrix();
         rotation.setEuler(0.0, -idealAngleBetweenGroundAndPlane *(Math.PI/180), 0.0); //2
         Point3D translation = new Point3D(0.0, 0.0, 0.0);
         AffineTransform transform = new AffineTransform(rotation, translation); 

         max.applyTransform(transform);
         min.applyTransform(transform);
         normalI.applyTransform(transform);        
         
         //distinguishing between floor and stair 
         double pointToGround = (min.getX() + max.getX()) / -2.0; // average + inverting sign
         //System.out.println(pointToGround);
         if(pointToGround > distanceCameraGround - MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE && pointToGround < distanceCameraGround + MIN_X_EXPECTED_X_DIFFERENCE_TOLERANCE)
            continue;         
         
         if(distance > min.getZ())
            distance =  min.getZ();
         
         double stairHeight = distanceCameraGround - pointToGround;
      }     
      
      return distance;        
   }   
   /*
    * returns DEFAULT_DISTANCE_VALUE if there is no obstacle otherwise return distance to obstacle
    * OBSOLETE
    */
   private double obstacleDistance(PlanarRegionsList planarRegionsList) {
      //variables
      double distance = DEFAULT_VALUE;  
      LinkedList<Double> distanceList = new LinkedList<Double>();
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         Vector3D vector = planarRegion.getNormal();
         double angle = Math.acos(vector.getZ())*(180/Math.PI);
         if(angle > 180 - ANGLE_CAMERA_PLANE_TOLERANCE) {
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