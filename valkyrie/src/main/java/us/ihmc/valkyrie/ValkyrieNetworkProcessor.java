package us.ihmc.valkyrie;

import static us.ihmc.avatar.networkProcessor.DRCNetworkProcessor.tryToStartModule;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxModule;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageLogger;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.reaStateUpdater.HumanoidAvatarREAStateUpdater;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.avatar.networkProcessor.walkingPreview.WalkingControllerPreviewToolboxModule;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;

public class ValkyrieNetworkProcessor
{
   public static final boolean launchFootstepPlannerModule = true;
   private static final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT);
   private static final String REAConfigurationFilePath = System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt";

   private final List<CloseableAndDisposable> modules = new ArrayList<>();
   private final PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
   private final String[] programArguments;
   private final URI rosURI = NetworkParameters.getROSURI();

   public ValkyrieNetworkProcessor(String[] args)
   {
      this.programArguments = args;

      System.out.println("ROS_MASTER_URI=" + rosURI);

      if (rosURI != null)
      {
         tryToStartModule(this::setupRosModule);
         tryToStartModule(this::setupSensorModule);
      }

      tryToStartModule(this::setupAStarFootstepPlanner);
      if (launchFootstepPlannerModule)
         tryToStartModule(this::setupFootstepPlanningToolboxModule);
      tryToStartModule(this::setupWalkingPreviewModule);
      tryToStartModule(this::setupBipedalSupportPlanarRegionPublisherModule);
      tryToStartModule(this::setupHumanoidAvatarREAStateUpdater);
      tryToStartModule(this::setupRobotEnvironmentAwerenessModule);
      tryToStartModule(this::setupKinematicsToolboxModule);
      tryToStartModule(this::setupKinematicsPlanningToolboxModule);
      tryToStartModule(this::setupKinematicsStreamingToolboxModule);

      LogTools.info("All modules in network processor are up and running!");
   }

   private void setupRosModule() throws IOException
   {
      modules.add(new RosModule(robotModel, rosURI, null, pubSubImplementation));
   }

   private void setupSensorModule() throws IOException
   {
      DRCSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
      sensorSuiteManager.initializePhysicalSensors(rosURI);
      sensorSuiteManager.connect();
      modules.add(sensorSuiteManager);
   }

   private void setupAStarFootstepPlanner()
   {
      ValkyrieAStarFootstepPlanner footstepPlanner = new ValkyrieAStarFootstepPlanner(robotModel);
      modules.add(footstepPlanner);
      footstepPlanner.setupWithRos(pubSubImplementation);
   }

   private void setupFootstepPlanningToolboxModule() throws IOException
   {
      modules.add(new MultiStageFootstepPlanningModule(robotModel, null, false, pubSubImplementation));
   }

   private void setupWalkingPreviewModule() throws IOException
   {
      modules.add(new WalkingControllerPreviewToolboxModule(robotModel, false, pubSubImplementation));
   }

   private void setupBipedalSupportPlanarRegionPublisherModule()
   {
      BipedalSupportPlanarRegionPublisher module = new BipedalSupportPlanarRegionPublisher(robotModel, pubSubImplementation);
      module.start();
      modules.add(module);
   }

   private void setupHumanoidAvatarREAStateUpdater()
   {
      modules.add(new HumanoidAvatarREAStateUpdater(robotModel, pubSubImplementation));
   }

   private void setupRobotEnvironmentAwerenessModule() throws Exception
   {
      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(REAConfigurationFilePath);
      modules.add(remoteModule);
      remoteModule.start();
   }

   private void setupKinematicsToolboxModule() throws IOException
   {
      modules.add(new KinematicsToolboxModule(robotModel, false, pubSubImplementation));
   }

   private void setupKinematicsPlanningToolboxModule() throws IOException
   {
      modules.add(new KinematicsPlanningToolboxModule(robotModel, false, pubSubImplementation));
   }

   private void setupKinematicsStreamingToolboxModule() throws IOException
   {
      JavaProcessSpawner javaProcessSpawner = new JavaProcessSpawner(true, true);
      Process process = javaProcessSpawner.spawn(ValkyrieKinematicsStreamingToolboxModule.class, programArguments);
      modules.add(new KinematicsStreamingToolboxMessageLogger(robotModel.getSimpleRobotName(), pubSubImplementation));
      modules.add(() ->
      {
         javaProcessSpawner.kill(process);
         javaProcessSpawner.shutdown();
      });
   }

   public void closeAndDispose()
   {
      for (CloseableAndDisposable module : modules)
      {
         try
         {
            module.closeAndDispose();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

      modules.clear();
   }

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      ValkyrieNetworkProcessor networkProcessor = new ValkyrieNetworkProcessor(args);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         networkProcessor.closeAndDispose();
         ThreadTools.sleep(10);
      }));
   }
}
