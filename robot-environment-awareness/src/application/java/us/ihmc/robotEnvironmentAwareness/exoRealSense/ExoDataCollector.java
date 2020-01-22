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

/*
 * Class for testing data recieving from exo and saving into files
 */
public class ExoDataCollector
{
   RealtimeRos2Node realTimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "ExoDataCollector");
   
   //signal flag for saving
   public AtomicReference<Boolean> saveIntoFiles = new AtomicReference<Boolean>(false);

   //conformation flags
   private boolean recievingLeftKneeHeight = false;
   private boolean recievingRightKneeHeight = false;
   private boolean recievingLeftThighAngle = false;
   private boolean recievingRightThighAngle = false;    

   //files for saving
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
      
      //subscribing to exo
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "mina_v2/knee_height/left"
                                           , this::handleExoLeftKneeHeight);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "mina_v2/knee_height/right"
                                           , this::handleExoRightKneeHeight);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "mina_v2/thigh_angle/left"
                                           , this::handleExoLeftThighAngle);
      ROS2Tools.createCallbackSubscription(realTimeRos2Node
                                           , Float64.class
                                           , "mina_v2/thigh_angle/right"
                                           , this::handleExoRightThighAngle);
   }

   private void handleExoLeftKneeHeight(Subscriber<Float64> subscriber) {
      //conformation message
      if(recievingLeftKneeHeight == false) {
         recievingLeftKneeHeight = true;
         System.out.println("recieving Left Knee Height");
      }

      //saving
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
      //conformation message
      if(recievingRightKneeHeight == false) {
         recievingRightKneeHeight = true;
         System.out.println("recieving Right Knee Height");
      }

      //saving
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
      //conformation message    
      if(recievingLeftThighAngle == false) {
         recievingLeftThighAngle = true;
         System.out.println("recieving Left Thigh Angle");
      }

      //saving
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
      //conformation message
      if(recievingRightThighAngle == false) {
         recievingRightThighAngle = true;
         System.out.println("recieving Right Thigh Angle");
      }

      //saving
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
      ExoDataCollector collector = new ExoDataCollector("EXO");
   }   
}
