package us.ihmc.ihmcPerception.perceptionLogger;

import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class PerceptionLogger
{
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                  getClass(),
                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   VideoPacket messageToLog = null;

   public PerceptionLogger()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new ROS2Callback<>(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT, message ->
      {
         LogTools.info("Message Received: ", message);
         messageToLog = message;
      });

      executorService.scheduleAtFixedRate(this::logVideoPacket, 0, 32, TimeUnit.MILLISECONDS);
   }

   private void logVideoPacket()
   {
      LogTools.info("Message Being Logged: ", messageToLog);
   }

   public static void main(String[] args)
   {
      new PerceptionLogger();
   }
}
