package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDebrisEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspPieceOfDebrisBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.UnfinishedTest;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

@UnfinishedTest
public abstract class DRCGraspPieceOfDebrisBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private ReferenceFrame midFeetZUpFrame;

   private void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final double FINGER1_JOINT_1_EXPECTED_RADIANS = 0.77;
   private final double FINGER1_JOINT_2_EXPECTED_RADIANS = 1.24;
   private final double FINGER1_JOINT_3_EXPECTED_RADIANS = 0.21;

   private final double FINGER2_JOINT_1_EXPECTED_RADIANS = 0.78;
   private final double FINGER2_JOINT_2_EXPECTED_RADIANS = 1.26;
   private final double FINGER2_JOINT_3_EXPECTED_RADIANS = 0.24;

   private final double MIDDLEFINGER_JOINT_1_EXPECTED_RADIANS = 0.96;
   private final double MIDDLEFINGER_JOINT_2_EXPECTED_RADIANS = 1.38;
   private final double MIDDLEFINGER_JOINT_3_EXPECTED_RADIANS = 1.06;

   private final double FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS = 0.2;

   private final double POSITION_ERROR_MARGIN = 0.025;
   private final double ANGLE_ERROR_MARGIN = 0.05;

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();

   private DoubleYoVariable yoTime;

   private SDFRobot robot;
   private SDFFullRobotModel fullRobotModel;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      showMemoryUsageBeforeTest();

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            Vector3d additionalOffset = new Vector3d(0.0, 0.0, 0.0);
            double yaw = 0.0;
            OffsetAndYawRobotInitialSetup offsetAndYawRobotInitialSetup = new OffsetAndYawRobotInitialSetup(additionalOffset, yaw);
            return offsetAndYawRobotInitialSetup;
         }
      };

            testEnvironment.addStandingDebris(0.75, -0.2, 0.0);
//      testEnvironment.addHorizontalDebrisLeaningOnTwoBoxes(new Point3d(0.75, 0.0, 0.9), Math.toRadians(0.0), Math.toRadians(110.0)); // 20 ,-20

      testEnvironment.createDebrisContactController();

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null, startingLocation,
            simulationTestingParameters, getRobotModel(), controllerCommunicator);

      yoTime = drcBehaviorTestHelper.getRobot().getYoTime();

      robot = drcBehaviorTestHelper.getRobot();
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      midFeetZUpFrame = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      
     
   }

   @AverageDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testGraspingDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      //angles in AtlasDefaultArmConfigurations
      //angles in AtlasDefaultArmConfigurations
      //right arm
      robot.getOneDegreeOfFreedomJoint("r_arm_shz").setQ(-0.1);
      robot.getOneDegreeOfFreedomJoint("r_arm_shx").setQ(1.6);
      robot.getOneDegreeOfFreedomJoint("r_arm_ely").setQ(1.9);
      robot.getOneDegreeOfFreedomJoint("r_arm_elx").setQ(-2.1);
      robot.getOneDegreeOfFreedomJoint("r_arm_wry").setQ(0.0);
      robot.getOneDegreeOfFreedomJoint("r_arm_wrx").setQ(0.55);
      //left arm
      robot.getOneDegreeOfFreedomJoint("l_arm_shz").setQ(0.1);
      robot.getOneDegreeOfFreedomJoint("l_arm_shx").setQ(-1.6);
      robot.getOneDegreeOfFreedomJoint("l_arm_ely").setQ(1.9);
      robot.getOneDegreeOfFreedomJoint("l_arm_elx").setQ(2.1);
      robot.getOneDegreeOfFreedomJoint("l_arm_wry").setQ(0.0);
      robot.getOneDegreeOfFreedomJoint("l_arm_wrx").setQ(-0.55);

      double[] rightArmDefaultConfigurationJointAngles = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, RobotSide.RIGHT);
      double[] leftArmDefaultConfigurationJointAngles = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, RobotSide.LEFT);

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.2));

      drcBehaviorTestHelper.sendHandPosePacketToListeners(new HandPosePacket(RobotSide.RIGHT, 0.5, rightArmDefaultConfigurationJointAngles));
      drcBehaviorTestHelper.sendHandPosePacketToListeners(new HandPosePacket(RobotSide.LEFT, 0.5, leftArmDefaultConfigurationJointAngles));
      drcBehaviorTestHelper.updateRobotModel();

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      drcBehaviorTestHelper.updateRobotModel();
      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, midFeetZUpFrame, getRobotModel(), yoTime);

      graspPieceOfDebrisBehavior.initialize();

      // from DebrisTaskBehaviorPanel.storeDebrisDataInList
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);

      //this offset is very important because debrisTransform sent from the UI have the origin at the bottom of the debris, whereas here the robots have their origin at the center of the debris
      double zOffsetToHaveOriginAtDebrisBottom = testEnvironment.getDebrisLength() / 2.0;

      RigidBodyTransform debrisTransform = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransform);

      PoseReferenceFrame debrisReferenceFrame = new PoseReferenceFrame("debrisReferenceFrame", worldFrame);
      debrisReferenceFrame.setPoseAndUpdate(debrisTransform);

      FramePose debrisPose = new FramePose(debrisReferenceFrame);
      debrisPose.setZ(-zOffsetToHaveOriginAtDebrisBottom);
      debrisPose.changeFrame(worldFrame);

      FrameVector graspVector = new FrameVector(debrisReferenceFrame);
      graspVector.set(-1.0, 0.0, 0.0);
      graspVector.changeFrame(worldFrame);

      FramePoint graspVectorPosition = new FramePoint(debrisReferenceFrame);
      graspVectorPosition.setZ(0.8 - zOffsetToHaveOriginAtDebrisBottom);
      graspVectorPosition.changeFrame(worldFrame);

      debrisPose.getRigidBodyTransform(debrisTransform);

      graspPieceOfDebrisBehavior.setGraspPose(debrisTransform, graspVectorPosition.getPointCopy(), graspVector.getVectorCopy(), RobotSide.RIGHT);

      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());

      double graspTime = 16.0;
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(graspPieceOfDebrisBehavior, graspTime);

      assertTrue(graspPieceOfDebrisBehavior.isDone());

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      DoubleYoVariable finger1_Joint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_1");
      DoubleYoVariable finger1_Joint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_2");
      DoubleYoVariable finger1_Joint3 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_3");

      DoubleYoVariable finger2_Joint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_1");
      DoubleYoVariable finger2_Joint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_2");
      DoubleYoVariable finger2_Joint3 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_3");

      DoubleYoVariable fingerMiddle_Joint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_middle_joint_1");
      DoubleYoVariable fingerMiddle_Joint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_middle_joint_2");
      DoubleYoVariable fingerMiddle_Joint3 = (DoubleYoVariable) robot.getVariable("q_r_finger_middle_joint_3");

      if (DEBUG)
      {
         System.out.println(finger1_Joint1);
         System.out.println(finger1_Joint2);
         System.out.println(finger1_Joint3);

         System.out.println(finger2_Joint1);
         System.out.println(finger2_Joint2);
         System.out.println(finger2_Joint3);

         System.out.println(fingerMiddle_Joint1);
         System.out.println(fingerMiddle_Joint2);
         System.out.println(fingerMiddle_Joint3);
      }
      assertTrue(Math.abs(finger1_Joint1.getDoubleValue() - FINGER1_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger1_Joint2.getDoubleValue() - FINGER1_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger1_Joint3.getDoubleValue() - FINGER1_JOINT_3_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);

      assertTrue(Math.abs(finger2_Joint1.getDoubleValue() - FINGER2_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger2_Joint2.getDoubleValue() - FINGER2_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger2_Joint3.getDoubleValue() - FINGER2_JOINT_3_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);

      assertTrue(Math.abs(fingerMiddle_Joint1.getDoubleValue() - MIDDLEFINGER_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(fingerMiddle_Joint2.getDoubleValue() - MIDDLEFINGER_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(fingerMiddle_Joint3.getDoubleValue() - MIDDLEFINGER_JOINT_3_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);

      //Right wrist Position
      ReferenceFrame rightHandReferenceFrame = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM).getBodyFixedFrame();
      FramePose rightHandPose = new FramePose(rightHandReferenceFrame);
      rightHandPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("right hand pose");
         System.out.println(rightHandPose);
      }
      FramePose rightHandExpectedPose = new FramePose(worldFrame);
      rightHandExpectedPose.setPosition(0.47, -0.21, 1.20);
      rightHandExpectedPose.setOrientation(1.56, 0.0, 0.13);
      assertTrue(rightHandPose.epsilonEquals(rightHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      //Left wrist Position
      ReferenceFrame leftHandReferenceFrame = fullRobotModel.getEndEffector(RobotSide.LEFT, LimbName.ARM).getBodyFixedFrame();
      FramePose leftHandPose = new FramePose(leftHandReferenceFrame);
      leftHandPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("left hand pose");
         System.out.println(leftHandPose);
      }
      FramePose leftHandExpectedPose = new FramePose(worldFrame);
      leftHandExpectedPose.setPosition(0.33, 0.24, 1.06);
      leftHandExpectedPose.setOrientation(-1.78, 0.0, 0.0);
      assertTrue(leftHandPose.epsilonEquals(leftHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      //Chest orientation
      ReferenceFrame chestReferenceFrame = fullRobotModel.getChest().getBodyFixedFrame();
      FramePose chestPose = new FramePose(chestReferenceFrame);
      chestPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("chest pose");
         System.out.println(chestPose);
      }
      FramePose chestExpectedPose = new FramePose(worldFrame);
      chestExpectedPose.setPosition(-0.16, -0.02, 1.29);
      chestExpectedPose.setOrientation(0.0, 0.0, 0.0);
      assertTrue(chestPose.epsilonEquals(chestExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      //pelvis pose
      ReferenceFrame pelvisReferenceFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FramePose pelvisPose = new FramePose(pelvisReferenceFrame);
      pelvisPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("pelvis pose");
         System.out.println(pelvisPose);
      }

      FramePose pelvisExpectedPose = new FramePose(worldFrame);
      pelvisExpectedPose.setPosition(-0.09, -0.01, 0.79);
      pelvisExpectedPose.setOrientation(0.0, 0.0, 0.0);
      assertTrue(pelvisPose.epsilonEquals(pelvisExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      BambooTools.reportTestFinishedMessage();
   }

   //TODO Test when the hand has to roll 180 degrees to grab a debris

   //TODO test the other hand
}
