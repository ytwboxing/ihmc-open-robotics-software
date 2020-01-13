package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.Console;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Random;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicReference;

import javax.imageio.ImageIO;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import std_msgs.msg.dds.Float64;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

/*
 * original class changed for testing purposes
 */
public class MultisenseStereoVisionPointCloudROS1Bridge extends AbstractRosTopicSubscriber<PointCloud2>
{
   private static final MultisenseInformation multisense = MultisenseInformation.CART;

   private static final int MAX_NUMBER_OF_POINTS = 200000;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");

   private IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   private static final int projectionWidth = 1024;
   private static final int projectionHeight = 544;

   private Scanner commandScanner;
   private static final String commandToSaveStereoVisionPointCloudData = "s";
   private static final String commandToStopSAvingStereoVisionPointCloudData = "d";
   private static final String commandToSaveProjectedData = "p";
   private int savingIndex = 0;

   public AtomicReference<Boolean> saveStereoVisionPointCloud = new AtomicReference<Boolean>(false);
   private AtomicReference<Boolean> saveProjectedData = new AtomicReference<Boolean>(false);
   
   String savePath; 
   boolean selfStart;     
   public MultisenseStereoVisionPointCloudROS1Bridge(String topic, String savePath, RosMainNode rosMainNode, boolean selfStart) throws URISyntaxException
   {
      super(PointCloud2._TYPE);
      this.savePath = savePath;
      this.selfStart = selfStart;
      //URI masterURI = new URI(multisense.getAddress());
      //URI masterURI = new URI("http://192.168.137.2:11311");
      //RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      //rosMainNode.attachSubscriber(MultisenseInformation.getStereoVisionPointCloudTopicName(), this);
      rosMainNode.attachSubscriber(topic, this);

      if(selfStart) {
         rosMainNode.execute();

         stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
         
         commandScanner = new Scanner(System.in);
         Runnable inputReader = new Runnable()
         {
            @Override
            public void run()
            {
               while (true)
               {
                  String command = commandScanner.next();

                  if (command.contains(commandToSaveStereoVisionPointCloudData))
                  {
                     saveStereoVisionPointCloud.set(true);
                     System.out.println(commandToSaveStereoVisionPointCloudData + " pressed");
                  }
                  else if (command.contains(commandToStopSAvingStereoVisionPointCloudData))
                  {
                     saveStereoVisionPointCloud.set(false);
                     System.out.println(commandToStopSAvingStereoVisionPointCloudData + " pressed");
                  }
                  else if (command.contains(commandToSaveProjectedData))
                  {
                     saveProjectedData.set(true);
                     System.out.println(commandToSaveProjectedData + " pressed");
                  }
               }
            }
         };
         Thread inputHolder = new Thread(inputReader);
         inputHolder.start();
      }
   }

   @Override
   public void onNewMessage(PointCloud2 cloudHolder)
   {
      //computeTimeRate();
      
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder);

      Point3D[] pointCloud = pointCloudData.getPoints();
      Color[] colors = pointCloudData.getPointColors();

      Random random = new Random();
      int numberOfPoints = pointCloud.length;

      while (numberOfPoints > MAX_NUMBER_OF_POINTS)
      {
         int indexToRemove = random.nextInt(numberOfPoints);
         int lastIndex = numberOfPoints - 1;

         pointCloud[indexToRemove] = pointCloud[lastIndex];
         colors[indexToRemove] = colors[lastIndex];

         numberOfPoints--;
      }

      long timestamp = cloudHolder.getHeader().getStamp().totalNsecs();
      if(selfStart) {
         float[] pointCloudBuffer = new float[3 * numberOfPoints];
         int[] colorsInteger = new int[numberOfPoints];

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
            pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
            pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

            colorsInteger[i] = colors[i].getRGB();
         }

         StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);

         stereoVisionPublisher.publish(stereoVisionMessage);      
      }      

      if (saveStereoVisionPointCloud.get())
      {
         FileWriter fileWriter;
         try
         {
            File f = new File(savePath + "/stereovision_pointcloud_" + savingIndex + ".txt");
            f.setLastModified(timestamp);
            fileWriter = new FileWriter(f);            
            StringBuilder builder = new StringBuilder("");
            for (int i = 0; i < numberOfPoints; i++)
            {
               Point3D scanPoint = pointCloud[i];
               builder.append(i + "\t" + scanPoint.getX() + "\t" + scanPoint.getY() + "\t" + scanPoint.getZ() + "\t" + colors[i].getRGB() + "\n");
            }
            fileWriter.write(builder.toString());
            fileWriter.close();
            //System.out.println(savingIndex);
         }
         catch (IOException e1)
         {
            e1.printStackTrace();
         }
         savingIndex++;
      }
      
      if (saveProjectedData.getAndSet(false))
      {
         BufferedImage bufferedImage = new BufferedImage(projectionWidth, projectionHeight, BufferedImage.TYPE_INT_RGB);

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];
            Point2D projectedPixel = new Point2D();
            int[] offset = multisense.getProjectionOffset();

            PointCloudProjectionHelper.projectMultisensePointCloudOnImage(scanPoint, projectedPixel, multisense.getIntrinsicParameters(), offset[0], offset[1]);

            boolean inImage = false;
            if (projectedPixel.getX() >= 0 && projectedPixel.getX() < projectionWidth)
               if (projectedPixel.getY() >= 0 && projectedPixel.getY() < projectionHeight)
                  inImage = true;

            if (inImage)
               bufferedImage.setRGB((int) projectedPixel.getX(), (int) projectedPixel.getY(), colors[i].getRGB());
         }

         File outputfile = new File("projected_image_" + savingIndex + ".png");
         try
         {
            ImageIO.write(bufferedImage, "png", outputfile);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         savingIndex++;
      }
   }

   int timeDistancesSize = 7;
   int[] timeDistances = new int[timeDistancesSize];
   int timeDistancesIndex = 0;
   long lastTime = System.currentTimeMillis();
   private void computeTimeRate() {
      long time = System.currentTimeMillis();
      timeDistances[timeDistancesIndex] = (int)(time - lastTime);
      lastTime = time;
      timeDistancesIndex++;
      if(timeDistancesIndex == timeDistancesSize)
         timeDistancesIndex = 0;
      int average = 0;
      for(int i = 0; i < timeDistancesSize; i++)
         average += timeDistances[i];
      average /= timeDistancesSize;
      System.out.println(this.savePath + average);
   }
   
   public static void main(String[] args) throws URISyntaxException
   {
      /*
      //self start      
      URI masterURI = new URI("http://10.7.4.52:11311"); //"http://192.168.0.12:11311"
      String dataSetNumber = "1";
      
      String path = "DATASETS/" + dataSetNumber;
      //new MultisenseStereoVisionPointCloudROS1Bridge("/cam_1/depth/color/points", path + "LPC", new RosMainNode(masterURI, "StereoVisionPublisher", true), true);
      new MultisenseStereoVisionPointCloudROS1Bridge("/cam_2/depth/color/points", path + "RPC", new RosMainNode(masterURI, "StereoVisionPublisher", true), true);
      */
      /*
      //one particular point cloud
      String dataset = "1"; //  "1 P2"
      String LR = "R"; //L
      String number = "3";
      File f = new File("DATASETS/" + dataset + "/" + LR + "PC/points (" + number + ").txt"); 
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
      IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      StereoVisionPointCloudMessage m = StereoVisionPointCloudDataLoader.getMessageFromFile(f); 

      BufferedReader reader1;
      BufferedReader reader2;
      double height = 0.0;
      double angle = 0.0;
      Float64 heightF;
      Float64 angleF;
      int index = Integer.valueOf(number);
      try
      {        
         reader1 = new BufferedReader(new FileReader(new File("DATASETS/" + dataset + "/EXO/" + LR + "KneeHeight.txt"))); 
         reader2 = new BufferedReader(new FileReader(new File("DATASETS/" + dataset + "/EXO/" + LR + "ThighAngle.txt")));
         while(index > 1) {
            reader1.readLine();  
            reader2.readLine(); 
            reader1.readLine();  
            reader2.readLine();  
            index--;
         }  

         reader1.readLine();
         reader2.readLine();  
         height = Double.valueOf(reader1.readLine()); 
         angle = Double.valueOf(reader2.readLine()); 
         
         heightF = new Float64();
         heightF.data_ = height;
         
         angleF = new Float64();
         angleF.data_ = angle;         
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
         return;
      }
      IHMCROS2Publisher<Float64> publisherHeight = ROS2Tools.createPublisher(ros2Node, Float64.class, "mina_v2/knee_height");
      IHMCROS2Publisher<Float64> publisherAngle = ROS2Tools.createPublisher(ros2Node, Float64.class, "mina_v2/thigh_angle");
      
      while(true) {
         stereoVisionPublisher.publish(m);
         publisherHeight.publish(heightF);
         publisherAngle.publish(angleF);
         try
         {
            Thread.sleep(2000);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }         
      }
      */
      
      
      //whole dataset
      System.out.println("waiting");
      new Scanner(System.in).next();
      System.out.println("started");
      
      String dataset = "1"; //  "1 P2"
      String LR = "R"; //L
      int publishTimes = 1;
      
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
      IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      File f = null; 
      StereoVisionPointCloudMessage m = null; 
      
      BufferedReader reader1 = null;
      BufferedReader reader2 = null;
      try
      {
         reader1 = new BufferedReader(new FileReader(new File("DATASETS/" + dataset + "/EXO/" + LR + "KneeHeight.txt")));
         reader2 = new BufferedReader(new FileReader(new File("DATASETS/" + dataset + "/EXO/" + LR + "ThighAngle.txt")));         
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
         return;
      }
      double height = 0.0;
      double angle = 0.0;
      Float64 heightF;
      Float64 angleF;
      int index = 1;
      int publishCounter = 0;

      while(true) {
         try
         {   
            f = new File("DATASETS/" + dataset + "/" + LR + "PC/points (" + index + ").txt");
            if(f == null || f.exists() == false) {
               //System.out.println("done");
              // return;
               
               index = 1;
               f = new File("DATASETS/" + dataset + "/" + LR + "PC/points (" + index + ").txt");    
               reader1 = new BufferedReader(new FileReader(new File("DATASETS/" + dataset + "/EXO/" + LR + "KneeHeight.txt")));
               reader2 = new BufferedReader(new FileReader(new File("DATASETS/" + dataset + "/EXO/" + LR + "ThighAngle.txt")));                 
            }
            m = StereoVisionPointCloudDataLoader.getMessageFromFile(f);             
   
            reader1.readLine();
            reader2.readLine();  
            height = Double.valueOf(reader1.readLine()); 
            angle = Double.valueOf(reader2.readLine()); 
            
            if(height == 500.0 || angle == 500.0) {
               index++;
               continue;
            }
            
            heightF = new Float64();
            heightF.data_ = height;
            
            angleF = new Float64();
            angleF.data_ = angle;         
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
            return;
         }
         IHMCROS2Publisher<Float64> publisherHeight = ROS2Tools.createPublisher(ros2Node, Float64.class, "mina_v2/knee_height");
         IHMCROS2Publisher<Float64> publisherAngle = ROS2Tools.createPublisher(ros2Node, Float64.class, "mina_v2/thigh_angle");
      
         while(publishCounter < publishTimes) {
            stereoVisionPublisher.publish(m);
            publisherHeight.publish(heightF);
            publisherAngle.publish(angleF);
            //System.out.println("publish " + index);
            try
            {
               Thread.sleep(000);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }  
            publishCounter++;
         }
         publishCounter = 0;
         
         try
         {
            Thread.sleep(10000);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }  
         index++;
      }
      
   }
   
}
