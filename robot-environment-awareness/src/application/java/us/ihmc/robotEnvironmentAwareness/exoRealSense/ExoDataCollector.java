package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.concurrent.atomic.AtomicReference;

import std_msgs.msg.dds.Float64;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.RealtimeRos2Node;

public class ExoDataCollector
{
   RealtimeRos2Node realTimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "ExoDataCollector");
   
   public AtomicReference<Boolean> saveIntoFiles = new AtomicReference<Boolean>(false);

   private boolean recievingLeftKneeHeight = false;
   private boolean recievingRightKneeHeight = false;
   private boolean recievingLeftThighAngle = false;
   private boolean recievingRightThighAngle = false;    

   BufferedWriter leftKneeHeightWriter;
   BufferedWriter rightKneeHeightWriter;
   BufferedWriter leftThighAngleWriter;
   BufferedWriter rightThighAngleWriter;   
   
   public ExoDataCollector(String savePath) 
   {
      try
      {
         leftKneeHeightWriter = new BufferedWriter(new FileWriter(savePath + "/LKneeHeight.txt")); 
         rightKneeHeightWriter = new BufferedWriter(new FileWriter(savePath + "/RKneeHeight.txt"));  
         leftThighAngleWriter = new BufferedWriter(new FileWriter(savePath + "/LThighAngle.txt"));   
         rightThighAngleWriter = new BufferedWriter(new FileWriter(savePath + "/RThighAngle.txt"));         
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
         return;
      }
      
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "knee_height/left" //mina_v2/
                                           , this::handleExoLeftKneeHeight);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "knee_height/right" //mina_v2/
                                           , this::handleExoRightKneeHeight);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "thigh_angle/left" //mina_v2/
                                           , this::handleExoLeftThighAngle);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "thigh_angle/right" //mina_v2/
                                           , this::handleExoRightThighAngle);
   }

   private void handleExoLeftKneeHeight(Subscriber<Float64> subscriber) {
      if(recievingLeftKneeHeight == false) {
         recievingLeftKneeHeight = true;
         System.out.println("recieving Left Knee Height");
      }

      if(saveIntoFiles.get()) {
         Double value = subscriber.takeNextData().data_;
         try
         {
            leftKneeHeightWriter.write(String.valueOf(System.currentTimeMillis()) + "\n");    
            leftKneeHeightWriter.write(String.valueOf(value) + "\n");
            leftKneeHeightWriter.flush();
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
         } 
      }         
   }    
   
   private void handleExoRightKneeHeight(Subscriber<Float64> subscriber) {
      if(recievingRightKneeHeight == false) {
         recievingRightKneeHeight = true;
         System.out.println("recieving Right Knee Height");
      }

      if(saveIntoFiles.get()) {
         Double value = subscriber.takeNextData().data_;
         try
         {
            rightKneeHeightWriter.write(String.valueOf(System.currentTimeMillis()) + "\n");    
            rightKneeHeightWriter.write(String.valueOf(value) + "\n");
            rightKneeHeightWriter.flush();
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
         }
      }
   }
   
   private void handleExoLeftThighAngle(Subscriber<Float64> subscriber) {      
      if(recievingLeftThighAngle == false) {
         recievingLeftThighAngle = true;
         System.out.println("recieving Left Thigh Angle");
      }

      if(saveIntoFiles.get()) {
         Double value = subscriber.takeNextData().data_;
         try
         {
            leftThighAngleWriter.write(String.valueOf(System.currentTimeMillis()) + "\n");    
            leftThighAngleWriter.write(String.valueOf(value) + "\n");
            leftThighAngleWriter.flush();
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
         }
      }
   }
   
   private void handleExoRightThighAngle(Subscriber<Float64> subscriber) {
      if(recievingRightThighAngle == false) {
         recievingRightThighAngle = true;
         System.out.println("recieving Right Thigh Angle");
      }
      
      if(saveIntoFiles.get()) {
         Double value = subscriber.takeNextData().data_;
         try
         {
            rightThighAngleWriter.write(String.valueOf(System.currentTimeMillis()) + "\n");    
            rightThighAngleWriter.write(String.valueOf(value) + "\n");
            rightThighAngleWriter.flush();
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
         }
      }
   }
   
   public static void main(String[] args)
   {          
      /*
      double value = 0.0;
      try { 
         BufferedWriter writer = new BufferedWriter(new FileWriter("test.txt"));
         while(true) {
            String fileContent = String.valueOf(System.currentTimeMillis()) + "\n";
            fileContent += String.valueOf(value++) + "\n";
            writer.write(fileContent);
            writer.flush();
            //writer.close(); 
            System.out.print(fileContent);
            Thread.sleep(1500);
         }
                 
      }
      catch(Exception ex) {
         ex.printStackTrace();
      }
      */
      /*
      try { 
         long myDelay = System.currentTimeMillis();
         File file = new File("test.txt");
         BufferedReader reader = new BufferedReader(new FileReader(file));         
         String line = reader.readLine();
         long time1 = 0;
         long time2 = file.lastModified() -5000;
         long artificialDelay = 500;
         while(true) {
            if(line == null || line.isEmpty()) {
               reader.close();
               reader = new BufferedReader(new FileReader(file));
               line = reader.readLine();               
               time2 = file.lastModified() -5000;              
            }
            
            time1 = Long.valueOf(line);
            double value = Double.valueOf(reader.readLine());
            
            long waitTime = time1 - time2 - (System.currentTimeMillis() - myDelay) + artificialDelay;
            if(waitTime > 0)
               Thread.sleep(waitTime);
            myDelay = System.currentTimeMillis();
            System.out.println(value);
            
            time2 = time1;            
            line = reader.readLine();
         }                 
      }
      catch(Exception ex) {
         ex.printStackTrace();
      }
      */
   }
   
}
