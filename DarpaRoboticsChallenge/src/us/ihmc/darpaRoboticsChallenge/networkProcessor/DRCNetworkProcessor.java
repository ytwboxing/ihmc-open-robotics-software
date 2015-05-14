package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map.Entry;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.MultisenseMocapManualCalibrationTestModule;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.RosModule;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiConnectionModule;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class DRCNetworkProcessor
{
   private static final IHMCCommunicationKryoNetClassList NET_CLASS_LIST = new IHMCCommunicationKryoNetClassList();
   private final PacketRouter<PacketDestination> packetRouter;
   private final boolean DEBUG = false;

   public DRCNetworkProcessor(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      packetRouter = new PacketRouter<>(PacketDestination.class);
      
      try
      {
         setupControllerCommunicator(params);
         setupUiModule(robotModel, params);
         setupSensorModule(robotModel, params);
         setupBehaviorModule(robotModel, params);
         setupHandModules(robotModel, params);
         setupRosModule(robotModel, params);
         setupGFEModule(params);
         setupMocapModule(robotModel, params);
         setupMultisenseManualTestModule(robotModel, params);
         addRobotSpecificModuleCommunicators(params.getRobotSpecificModuleCommunicatorPorts());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
 }

   private void addRobotSpecificModuleCommunicators(HashMap<NetworkPorts, PacketDestination> ports)
   {
      for (Entry<NetworkPorts, PacketDestination> entry : ports.entrySet())
      {
         NetworkPorts port = entry.getKey();
         PacketDestination destinationId = entry.getValue();
         
         PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(destinationId, packetCommunicator);
         try {
			packetCommunicator.connect();
		} 
        catch (IOException e) 
        {
			e.printStackTrace();
		}
         printModuleConnectedDebugStatement(destinationId, "addRobotSpecificModuleCommunicators");
      }
   }

   private void setupMultisenseManualTestModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      if (params.isMultisenseManualTestModuleEnabled())
      {
         new MultisenseMocapManualCalibrationTestModule(robotModel, params.getRosUri());

         PacketCommunicator multisenseModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.MULTISENSE_TEST_MODULE, multisenseModuleCommunicator);
         try
         {
            multisenseModuleCommunicator.connect();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         
         String methodName = "setupMultisenseManualTestModule";
         printModuleConnectedDebugStatement(PacketDestination.MULTISENSE_TEST_MODULE, methodName);
      }
   }

   private void setupMocapModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)  throws IOException
   {
     if (params.isMocapModuleEnabled())
     {
//        new MocapDetectedObjectModule(null, null, null);

        PacketCommunicator mocapModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE, NET_CLASS_LIST);
        packetRouter.attachPacketCommunicator(PacketDestination.MOCAP_MODULE, mocapModuleCommunicator);
        mocapModuleCommunicator.connect();
        
        String methodName = "setupMocapModule";
        printModuleConnectedDebugStatement(PacketDestination.MOCAP_MODULE, methodName);
     }
   }

   private void setupHandModules(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isHandModuleEnabled())
      {
         SideDependentList<? extends HandCommandManager> handCommandModule = robotModel.createHandCommandManager();
         if (handCommandModule == null)
         {
            return;
         }
         
         for(RobotSide robotSide : RobotSide.values)
         {
            if(handCommandModule.get(robotSide) != null)
            {
               NetworkPorts port = robotSide == RobotSide.LEFT ? NetworkPorts.LEFT_HAND_MANAGER_PORT : NetworkPorts.RIGHT_HAND_MANAGER_PORT;
               PacketCommunicator handModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port, NET_CLASS_LIST);
               PacketDestination destination = robotSide == RobotSide.LEFT ? PacketDestination.LEFT_HAND : PacketDestination.RIGHT_HAND;
               packetRouter.attachPacketCommunicator(destination, handModuleCommunicator);
               handModuleCommunicator.connect();
               
               String methodName = "setupHandModules ";
               printModuleConnectedDebugStatement(destination, methodName);
            }
         }
      }
   }

   private void setupBehaviorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isBehaviorModuleEnabled())
      {
         DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();

         if (params.isAutomaticDiagnosticEnabled())
         {
            IHMCHumanoidBehaviorManager.createBehaviorModuleForAutomaticDiagnostic(robotModel, logModelProvider, params.isBehaviorVisualizerEnabled(), sensorInformation, params.getTimeToWaitBeforeStartingDiagnostics());
         }
         else
         {
            new IHMCHumanoidBehaviorManager(robotModel, logModelProvider, params.isBehaviorVisualizerEnabled(), sensorInformation);
         }
         
         PacketCommunicator behaviorModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorModuleCommunicator);
         behaviorModuleCommunicator.connect();


         String methodName = "setupBehaviorModule ";
         printModuleConnectedDebugStatement(PacketDestination.BEHAVIOR_MODULE, methodName);
      }
   }

   private void setupRosModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isRosModuleEnabled())
      {
         new RosModule(robotModel, params.getRosUri(), params.getSimulatedSensorCommunicator());

         PacketCommunicator rosModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_MODULE, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.ROS_MODULE, rosModuleCommunicator);
         rosModuleCommunicator.connect();
         
         String methodName = "setupRosModule ";
         printModuleConnectedDebugStatement(PacketDestination.ROS_MODULE, methodName);
      }
   }
   
   private void setupSensorModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isSensorModuleEnabled())
      {
         DRCSensorSuiteManager sensorSuiteManager = robotModel.getSensorSuiteManager();
         if (params.isSimulatedSensorsEnabled())
         {
            sensorSuiteManager.initializeSimulatedSensors(params.getSimulatedSensorCommunicator());
         }
         else
         {
            sensorSuiteManager.initializePhysicalSensors(params.getRosUri());
         }
         sensorSuiteManager.connect();
         
         PacketCommunicator sensorSuiteManagerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.SENSOR_MANAGER, sensorSuiteManagerCommunicator);
         sensorSuiteManagerCommunicator.connect();
         
         
         String methodName = "setupSensorModule ";
         printModuleConnectedDebugStatement(PacketDestination.SENSOR_MANAGER, methodName);
      }
   }

   private void setupUiModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isUiModuleEnabled())
      {
         new UiConnectionModule();
         
         PacketCommunicator uiModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.UI, uiModuleCommunicator);
         uiModuleCommunicator.connect();
         String methodName = "setupUiModule ";
         printModuleConnectedDebugStatement(PacketDestination.UI, methodName);
      }
   }

   private void setupGFEModule(DRCNetworkModuleParameters params) throws IOException
   {
      if(params.isGFECommunicatorEnabled())
      {
         PacketCommunicator gfeCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, NET_CLASS_LIST);
         packetRouter.attachPacketCommunicator(PacketDestination.GFE, gfeCommunicator);
         gfeCommunicator.connect();
         String methodName = "setupGFEModule ";
         printModuleConnectedDebugStatement(PacketDestination.GFE, methodName);
      }
   }

   private void setupControllerCommunicator(DRCNetworkModuleParameters params) throws IOException
   {
      if (params.isControllerCommunicatorEnabled())
      {
         PacketCommunicator controllerPacketCommunicator;
         if(params.isLocalControllerCommunicatorEnabled())
         {
            PrintTools.info(this, "Connecting to controller using intra process communication");
            controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, NET_CLASS_LIST);
         }
         else 
         {
            System.out.println("Connecting to controller using TCP on " + NetworkParameters.getHost(NetworkParameterKeys.robotController));
            controllerPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(NetworkParameters.getHost(NetworkParameterKeys.robotController), NetworkPorts.CONTROLLER_PORT, NET_CLASS_LIST);
         }
         
         packetRouter.attachPacketCommunicator(PacketDestination.CONTROLLER, controllerPacketCommunicator);
         controllerPacketCommunicator.connect();

         String methodName = "setupControllerCommunicator ";
         printModuleConnectedDebugStatement(PacketDestination.CONTROLLER, methodName);
      }
   }

   protected void connect(PacketCommunicator communicator)
   {
      try
      {
         communicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public PacketRouter<PacketDestination> getPacketRouter()
   {
      return packetRouter;
   }
   
   private void printModuleConnectedDebugStatement(PacketDestination destination, String methodName)
   {
      if (DEBUG)
      {
         PrintTools.debug(this, methodName + ": " + destination);
      }
   }
}
