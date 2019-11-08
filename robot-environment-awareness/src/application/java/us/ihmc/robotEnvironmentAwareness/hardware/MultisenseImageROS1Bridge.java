package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicReference;

import javax.imageio.ImageIO;

import org.jboss.netty.buffer.ChannelBuffer;

import controller_msgs.msg.dds.Image32;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Header;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class MultisenseImageROS1Bridge extends AbstractRosTopicSubscriber<Image>
{
   private static final MultisenseInformation multisense = MultisenseInformation.CART;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   //private final IHMCROS2Publisher<Image32> imagePublisher;

   //private final MultisenseCameraInfoROS1Bridge cameraInfoBridge;

   //private final Scanner commandScanner;
   private static final String commandToSaveImage = "s";
   private static final String commandToStopSavingImage = "d";
   private static final String commandToShowCameraInfo = "c";
   private int savingIndex = 0;

   public AtomicReference<Boolean> saveImage = new AtomicReference<Boolean>(false);
   private AtomicReference<Boolean> showCameraInfo = new AtomicReference<Boolean>(false);

   private String savePath;
   public MultisenseImageROS1Bridge(String topic, String savePath, RosMainNode rosMainNode) throws URISyntaxException, IOException
   {
      super(Image._TYPE);
      this.savePath = savePath;
      //commandScanner = new Scanner(System.in);
      //URI masterURI = new URI(multisense.getAddress());
      //URI masterURI = new URI("http://192.168.137.2:11311");
      //RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      //rosMainNode.attachSubscriber(MultisenseInformation.getImageTopicName(), this);
      rosMainNode.attachSubscriber(topic, this);      
      //rosMainNode.execute();

      //imagePublisher = ROS2Tools.createPublisher(ros2Node, Image32.class, ROS2Tools.getDefaultTopicNameGenerator());
      System.out.println(ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(Image32.class));

      //cameraInfoBridge = new MultisenseCameraInfoROS1Bridge();
      
      /*
      Runnable inputReader = new Runnable()
      {
         @Override
         public void run()
         {
            while (true)
            {
               String command = commandScanner.next();

               if (command.contains(commandToSaveImage))
               {
                  saveImage.set(true);
                  System.out.println(commandToSaveImage + " pressed");
               }
               else if (command.contains(commandToStopSavingImage))
               {
                  saveImage.set(false);
                  System.out.println(commandToStopSavingImage + " pressed");
               }
               else if (command.contains(commandToShowCameraInfo))
               {
                  showCameraInfo.set(true);
                  System.out.println(commandToShowCameraInfo + " pressed");
               }
            }
         }
      };
      Thread inputHolder = new Thread(inputReader);
      inputHolder.start();
      */
   }

   @Override
   public void onNewMessage(Image image)
   {
      int width = image.getWidth();
      int height = image.getHeight();

      //Image32 message = new Image32();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

      //message.setHeight(height);
      //message.setWidth(width);

      ChannelBuffer data = image.getData();
      byte[] array = data.array();
      int dataIndex = data.arrayOffset();
       /*     
      int minX = 214;
      int maxX = 426;
      int minY = 160;
      int maxY = 320;
      */
      ByteBuffer wrapped = null;
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {    
            byte a1 = array[dataIndex];            
            dataIndex++;
            byte a2 = array[dataIndex]; 
            dataIndex++; 
            
            wrapped = ByteBuffer.wrap(new byte[] {a2, a1}); 
            int value = wrapped.getShort(); 
            bufferedImage.setRGB(j, i, value);
            /*
            int rgbColor = 0;            
            if((j == minX || j == maxX)
                  && i >= minY && i <= maxY) {
               rgbColor = convertBGR2RGB(0,255,255);
            }
            else if((i == minY || i == maxY)
                  && j >= minX && j <= maxX) {
                  rgbColor = convertBGR2RGB(0,255,255);  
            }
            else {
               if(value < 4096 && value > 0) { 
                  value *= 255.0/4095.0;
                  rgbColor = convertBGR2RGB(0, value, 255-value);             
               }                  
            } 
            message.getRgbdata().add(rgbColor);
            */
            /*            
            int getValue = bufferedImage.getRGB(j, i);
            byte[] getArray = ByteBuffer.allocate(4).putInt(getValue).array();
            */
         }
      }

      //imagePublisher.publish(message);      
      
      if (saveImage.get())
      {
         File outputfile = new File(savePath + "/image_" + savingIndex + ".jpg");
         try
         {
            ImageIO.write(bufferedImage, "jpg", outputfile);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         savingIndex++;
         //System.out.println(savePath + " - " + savingIndex);
      }
   }

   private static int convertBGR2RGB(int b, int g, int r)
   {
      int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
      return rgb;
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      /*
      long sleepTime;
      long lastModified = new File("DATASETS/1/LPC/stereovision_pointcloud_0.txt").lastModified();
      long myDelay = System.currentTimeMillis();
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
      IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      int i = 0;
      while(true) {
         File f = new File("DATASETS/1/LPC/stereovision_pointcloud_" + i + ".txt");   
         if(f.exists() == false) {
            i = 0;
            System.out.println("again");
            continue;
            //break;            
         }
            
         StereoVisionPointCloudMessage m = StereoVisionPointCloudDataLoader.getMessageFromFile(f); 
         
         sleepTime = f.lastModified() - lastModified - (System.currentTimeMillis() - myDelay);
         if(sleepTime < 0)
            sleepTime = 0;
         try
         {
            Thread.sleep(sleepTime);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
         System.out.println("published " + i);
         stereoVisionPublisher.publish(m);
         myDelay = System.currentTimeMillis();
         lastModified = f.lastModified();
         i++;
      }
      */
      
      
      /*
      URI masterURI = new URI("http://192.168.137.2:11311");
      new MultisenseImageROS1Bridge("/cam_2/depth/image_rect_raw", "", new RosMainNode(masterURI, "whatever", true));  
      */
      
      /*
      String datasetNUmber = "1";
      URI masterURI = new URI("http://192.168.137.2:11311");
      RosMainNode node = new RosMainNode(masterURI, "whatever", true);
      MultisenseImageROS1Bridge leftI = new MultisenseImageROS1Bridge("/cam_2/depth/image_rect_raw", "DATASETS/" + datasetNUmber + "/LI", node);  
      MultisenseImageROS1Bridge rightI = new MultisenseImageROS1Bridge("/cam_1/depth/image_rect_raw", "DATASETS/" + datasetNUmber + "/RI", node); 
      MultisenseStereoVisionPointCloudROS1Bridge leftPC = new MultisenseStereoVisionPointCloudROS1Bridge("/cam_2/depth/color/points", "DATASETS/" + datasetNUmber + "/LPC", node);
      MultisenseStereoVisionPointCloudROS1Bridge rightPC = new MultisenseStereoVisionPointCloudROS1Bridge("/cam_1/depth/color/points", "DATASETS/" + datasetNUmber + "/RPC", node);

      node.execute();
      Scanner commandScanner = new Scanner(System.in);
      while (true)
      {
         String command = commandScanner.next();

         if (command.contains(commandToSaveImage))
         {
            leftI.saveImage.set(true);
            rightI.saveImage.set(true);
            leftPC.saveStereoVisionPointCloud.set(true);
            rightPC.saveStereoVisionPointCloud.set(true);
            System.out.println(commandToSaveImage + " pressed");
         }
         else if (command.contains(commandToStopSavingImage))
         {
            leftI.saveImage.set(false);
            rightI.saveImage.set(false);
            leftPC.saveStereoVisionPointCloud.set(false);
            rightPC.saveStereoVisionPointCloud.set(false);
            System.out.println(commandToStopSavingImage + " pressed");
         }
      }
      */
   }

   private class MultisenseCameraInfoROS1Bridge extends AbstractRosTopicSubscriber<CameraInfo>
   {
      public MultisenseCameraInfoROS1Bridge() throws URISyntaxException, IOException
      {
         super(CameraInfo._TYPE);
         //URI masterURI = new URI(multisense.getAddress());
         URI masterURI = new URI("http://192.168.137.2:11311");
         RosMainNode rosMainNode = new RosMainNode(masterURI, "CameraInfoPublisher", true);
         //rosMainNode.attachSubscriber(MultisenseInformation.getCameraInfoTopicName(), this);
         rosMainNode.attachSubscriber("/cam_2/depth/camera_info", this);
         rosMainNode.execute();
      }

      @Override
      public void onNewMessage(CameraInfo cameraInfo)
      {
         if (showCameraInfo.getAndSet(false))
         {
            System.out.println("## CameraInfo");
            String distortionModel = cameraInfo.getDistortionModel();
            System.out.println(distortionModel);
            double[] d = cameraInfo.getD();
            double[] k = cameraInfo.getK();
            double[] p = cameraInfo.getP();
            double[] r = cameraInfo.getR();
            Header header = cameraInfo.getHeader();
            int height = cameraInfo.getHeight();
            int width = cameraInfo.getWidth();

            System.out.println("D");
            for (int i = 0; i < d.length; i++)
               System.out.println(d[i]);

            System.out.println("K");
            for (int i = 0; i < k.length; i++)
               System.out.println(k[i]);

            System.out.println("P");
            for (int i = 0; i < p.length; i++)
               System.out.println(p[i]);

            System.out.println("R");
            for (int i = 0; i < r.length; i++)
               System.out.println(r[i]);

            System.out.println(header.getStamp() + " " + header.getFrameId());
            System.out.println("height = " + height + " width = " + width);
         }
      }
   }
}
