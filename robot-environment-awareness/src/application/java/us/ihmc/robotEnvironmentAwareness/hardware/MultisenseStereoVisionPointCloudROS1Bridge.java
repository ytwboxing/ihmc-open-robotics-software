package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
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
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
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

      if(selfStart) {
         long timestamp = cloudHolder.getHeader().getStamp().totalNsecs();
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
            fileWriter = new FileWriter(savePath + "/stereovision_pointcloud_" + savingIndex + ".txt");
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

   public static void main(String[] args) throws URISyntaxException
   {
      /*
      //one particular point cloud
      File f = new File("DATASETS/2/RPC/points (53).txt");
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
      IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      StereoVisionPointCloudMessage m = StereoVisionPointCloudDataLoader.getMessageFromFile(f); 
      while(true) {
         stereoVisionPublisher.publish(m);
         
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
      
      /*
      //whole data set
      final String number = "2";
      final String leftRight = "R";
      final long artificialDelay = 1000;
      
      long sleepTime;
      long lastModified = new File("DATASETS/" + number + "/" + leftRight + "PC/points (1).txt").lastModified();
      long myDelay = System.currentTimeMillis();
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
      IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      int i = 40;
      while(true) {
         File f = new File("DATASETS/" + number + "/" + leftRight + "PC/points (" + i + ").txt");   
         if(f.exists() == false) {
            if(i == 1) {
               System.out.println("nothing to load!");
               break;
            }
            i = 1;
            //System.out.println("again");
            continue;
            //break;            
         }
            
         StereoVisionPointCloudMessage m = StereoVisionPointCloudDataLoader.getMessageFromFile(f); 
         
         sleepTime = f.lastModified() - lastModified - (System.currentTimeMillis() - myDelay) + artificialDelay;
         if(sleepTime > 0) {
            try
            {
               Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }            
         }
         stereoVisionPublisher.publish(m);
         //System.out.println("published " + i);
         myDelay = System.currentTimeMillis();
         lastModified = f.lastModified();
         i++;
      }            
      */
      
      
      //self start      
      URI masterURI = new URI("http://192.168.137.2:11311");
      new MultisenseStereoVisionPointCloudROS1Bridge("/cam_2/depth/color/points", "", new RosMainNode(masterURI, "StereoVisionPublisher", true), true);
      
      
      
   }
}
