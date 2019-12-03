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

/*
 * original class changed for testing purposes
 * static main contains data set creating and loading 
 */
public class MultisenseImageROS1Bridge extends AbstractRosTopicSubscriber<Image>
{
   private static final MultisenseInformation multisense = MultisenseInformation.CART;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   private IHMCROS2Publisher<Image32> imagePublisher;

   //private final MultisenseCameraInfoROS1Bridge cameraInfoBridge;

   private Scanner commandScanner;
   private static final String commandToSaveImage = "s";
   private static final String commandToStopSavingImage = "d";
   private static final String commandToShowCameraInfo = "c";
   private int savingIndex = 0;

   public AtomicReference<Boolean> saveImage = new AtomicReference<Boolean>(false);
   private AtomicReference<Boolean> showCameraInfo = new AtomicReference<Boolean>(false);

   private String savePath;
   private boolean selfStart;
   public MultisenseImageROS1Bridge(String topic, String savePath, RosMainNode rosMainNode, boolean selfStart) throws URISyntaxException, IOException
   {
      super(Image._TYPE);
      this.savePath = savePath;
      this.selfStart = selfStart;
      commandScanner = new Scanner(System.in);
      //URI masterURI = new URI(multisense.getAddress());
      //URI masterURI = new URI("http://192.168.137.2:11311");
      //RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      //rosMainNode.attachSubscriber(MultisenseInformation.getImageTopicName(), this);
      rosMainNode.attachSubscriber(topic, this);      

      System.out.println(ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(Image32.class));

      //cameraInfoBridge = new MultisenseCameraInfoROS1Bridge();

      if(selfStart) {
         rosMainNode.execute();
         imagePublisher = ROS2Tools.createPublisher(ros2Node, Image32.class, ROS2Tools.getDefaultTopicNameGenerator());
         
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
      }  
   }

   private final int VALUE_FOR_WHITE = 32767;
   
   @Override
   public void onNewMessage(Image image)
   {
      //original
      int width = image.getWidth();
      int height = image.getHeight();

      Image32 message = new Image32();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

      message.setHeight(height);
      message.setWidth(width);

      ChannelBuffer data = image.getData();
      byte[] array = data.array();
      int dataIndex = data.arrayOffset();
          
      int minX = 214;
      int maxX = 426;
      int minY = 160;
      int maxY = 320;
      
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
            
            if(selfStart) {
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
            }                 
         }
      }

      if(selfStart) {
         imagePublisher.publish(message);          
      }     
      
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
         System.out.println(savePath + " - " + savingIndex);
      }
      
      
      /*
      int width = image.getWidth();
      int height = image.getHeight();

      Image32 message = new Image32();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

      message.setWidth(width);
      message.setHeight(height);

      ChannelBuffer data = image.getData();
      byte[] array = data.array();
      int dataIndex = data.arrayOffset();          
      
      ByteBuffer wrapped = null;
      short[][] values = new short[width][height];
      for (int y = 0; y < height; y++)
      { 
         for (int x = 0; x < width; x++)
         {   
            byte a1 = array[dataIndex];            
            dataIndex++;
            byte a2 = array[dataIndex]; 
            dataIndex++; 
            
            wrapped = ByteBuffer.wrap(new byte[] {a2, a1}); 
            short value = wrapped.getShort(); 
            bufferedImage.setRGB(x, y, (int)value);   
            
            values[x][y] = value;
         }
      }
      
      //findABCForEquation(values, width/2, height/2); 
      values = detectEdges(values);

      for (int y = 0; y < height; y++)
      {  
         for (int x = 0; x < width; x++)
         {  
            short value = values[x][y];
            int rgbColor = 0; 
            
            if(value < 4096 && value > 0) { 
               value *= 255.0/4095.0;
               rgbColor = convertBGR2RGB(0, value, 255-value);             
            }
            else if(value == VALUE_FOR_WHITE) {
               rgbColor = 16777215;
            }
            message.getRgbdata().add(rgbColor);     
         }
      }

      if(selfStart) {
         imagePublisher.publish(message);          
      }     
      
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
         System.out.println(savePath + " - " + savingIndex);
      }*/
   }
   
   private final short DIFFERENCE_TOLLERANCE = 2;
   private short[][] detectEdges(short[][] values)
   {
      int width = values.length;
      int height = values[0].length;
      short[][] valuesNew = new short[width][height];

      for(int i = 1; i < width-1; i++) {
         for(int j = 1; j < height-1; j++) {
            //valuesNew[i][j] = values[i][j];
            
            short xy = values[i][j];
            short xm1y = values[i-1][j];
            short xp1y = values[i+1][j];
            short xym1 = values[i][j-1];
            short xyp1 = values[i][j+1];
            
            if(values[i][j] == 0) { //0 edges
               if(xm1y != 0
                  || xp1y != 0
                  || xym1 != 0
                  || xyp1 != 0
                  ) {
                  valuesNew[i][j] = VALUE_FOR_WHITE;
               }
            }
            else {
               if(xm1y == 0
                  || xp1y == 0
                  || xym1 == 0
                  || xyp1 == 0) {
                  valuesNew[i][j] = values[i][j];
               }
               else {
                  //x
                  short a12 = (short) (xm1y - xy);
                  short a23 = (short) (xy - xp1y);
                  short aDifference = (short) Math.abs(a12 - a23);
                  
                  if(aDifference > DIFFERENCE_TOLLERANCE) {
                     valuesNew[i][j] = VALUE_FOR_WHITE;
                  }
                  else {
                     valuesNew[i][j] = values[i][j];                  
                  }
                  
                  //y
                  double b12 = xym1 - xy;
                  double b23 = xy - xyp1;
                  short bDifference = (short) Math.abs(b12 - b23);
                  
                  if(bDifference > DIFFERENCE_TOLLERANCE) {
                     valuesNew[i][j] = VALUE_FOR_WHITE;
                  }                
               }               
            }
         }
      }
      return valuesNew;
   }

   int searchRadius = 5;
   double percentageDifferenceTolerance = 0.2;
   private void findABCForEquation(short[][] values, int x, int y)
   {
      short c = values[x][y];
      if(c == 0)
         return;

      double a = 0.0;
      for(int i = x-searchRadius; i <= x+searchRadius; i++ ) {
         double distance = (double)i - x;
         if(distance == 0)
            continue;
         a += (values[i][y] - c)/distance;
      }
      a /= searchRadius*2.0;
      
      double b = 0.0; 
      for(int j = y-searchRadius; j <= y+searchRadius; j++) {
         double distance = (double)j - y;
         if(distance == 0)
            continue;
         b += (values[x][j] - c)/distance;
      }
      b /= searchRadius*2.0;
      
      
      double theoreticalValue = a*-10 + b*-10 + c;
      double percentageDifference = values[x-10][y-10]/theoreticalValue;
      if(percentageDifference > 1.0)
         percentageDifference -= 1.0;
      if(percentageDifference < percentageDifferenceTolerance) {
         System.out.println("yes");
         for(int i = x; i > x-10; i--) {
            for(int j = y; y > y-10; y--){
               values[i][j] = 32767;
            }
         }
      }
      //System.out.println(percentageDifference);
      
         
      /*
      double a = (values[x+10][y] - c)/10.0;
      double b = (values[x][y+10] - c)/10.0;      
      
      double check = 10*a + 10*b + c;
      double difference = Math.abs(check - values[x+10][y+10]);
      System.out.println(difference);
      if(difference < 1.0) {
         for(int i = x; i < x+10; i++) {
            for(int j = y; j < y+10; j++) {
               values[i][j] = 32767;
            }            
         }
      }*/
   }

   private static int convertBGR2RGB(int b, int g, int r)
   {
      int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
      return rgb;
   }   

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      /*
      //one particular image
      File f = new File("DATASETS/1/LI/image (1).jpg");
      BufferedImage img = ImageIO.read(f);      
      Image32 message = new Image32();
      
      ByteBuffer wrappedInt = null;
      ByteBuffer wrappedShort = null;
      
      int width = img.getWidth();
      int height = img.getHeight();
      message.setWidth(width);
      message.setHeight(height);
      
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {              
            message.getRgbdata().add(img.getRGB(j, i));                     
         }
      }
      
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ImagePublisher");
      IHMCROS2Publisher<Image32> imagePublisher = ROS2Tools.createPublisher(ros2Node, Image32.class, ROS2Tools.getDefaultTopicNameGenerator());      
      while(true) {
         imagePublisher.publish(message);
         
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
      //standalone
      URI masterURI = new URI("http://192.168.137.2:11311");
      new MultisenseImageROS1Bridge("/cam_2/depth/image_rect_raw/compressed", "", new RosMainNode(masterURI, "whatever", true), true);
      */      
      
      /*
      //dataset creator
      final String datasetNUmber = "2";
      URI masterURI = new URI("http://192.168.137.2:11311");
      RosMainNode node = new RosMainNode(masterURI, "whatever", true);
      
      MultisenseImageROS1Bridge leftI = new MultisenseImageROS1Bridge("/cam_1/color/image_raw", "DATASETS/" + datasetNUmber + "/LI", node, false);  
      //MultisenseImageROS1Bridge rightI = new MultisenseImageROS1Bridge("/cam_2/depth/image_rect_raw", "DATASETS/" + datasetNUmber + "/RI", node, false); 
      //MultisenseStereoVisionPointCloudROS1Bridge leftPC = new MultisenseStereoVisionPointCloudROS1Bridge("/cam_1/depth/color/points", "DATASETS/" + datasetNUmber + "/LPC", node, false);
      //MultisenseStereoVisionPointCloudROS1Bridge rightPC = new MultisenseStereoVisionPointCloudROS1Bridge("/cam_2/depth/color/points", "DATASETS/" + datasetNUmber + "/RPC", node, false);

      node.execute();
      Scanner commandScanner = new Scanner(System.in);
      while (true)
      {
         String command = commandScanner.next();

         if (command.contains(commandToSaveImage))
         {
            leftI.saveImage.set(true);
            //rightI.saveImage.set(true);
            //leftPC.saveStereoVisionPointCloud.set(true);
            //rightPC.saveStereoVisionPointCloud.set(true);
            System.out.println(commandToSaveImage + " pressed");
         }
         else if (command.contains(commandToStopSavingImage))
         {
            leftI.saveImage.set(false);
            //rightI.saveImage.set(false);
            //leftPC.saveStereoVisionPointCloud.set(false);
            //rightPC.saveStereoVisionPointCloud.set(false);
            System.out.println(commandToStopSavingImage + " pressed");
         }
      } */
   }
}
