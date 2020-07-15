package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.ArrayTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;

import javax.swing.*;

import java.awt.Color;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/*
 * Simulation of two sphere robots, one with the basic ICP controller, the other with the LQR controller for tracking DCM
 */

public class SimpleLQRMomentumControllerSimulation
{
   private final YoVariableRegistry registry = new YoVariableRegistry("test");
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private static final boolean include1 = true;
   private static final boolean include2 = true;

   private static boolean visualize = true;

   private final static double nominalHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;
   private final static double NextRobotOffset = 1;

   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 10.0;
   private static final double stepDuration = 0.7;
   private static final double swingDuration = 0.5;
   private static final double stanceDuration = 0.2;
   private static final double stepLength = 0.5;
   private static final double stepWidth = 0.3;
   private static final int numberOfSteps = 10;

   private final static int NumRobots = 1;

   private final SimulationConstructionSet scs;
   
   private final Graphics3DObject worldGraphics = new Graphics3DObject();
   private final SideDependentList<MovingReferenceFrame> soleFrames = createSoleFrames();
   
   private final YoDouble yoTime = new YoDouble("time", registry);
   private final ExecutionTimer stopwatch = new ExecutionTimer("timer", 0.0, registry);

   private static List<Vector3D> initialPositions = new ArrayList<>();
   private static List<SimpleSphereControllerInterface> sphereControllers = new ArrayList<>();
   private static List<SimpleBipedCoMTrajectoryPlanner> dcmPlans = new ArrayList<>();

   public SimpleLQRMomentumControllerSimulation()
   {
      List<SimpleSphereRobot> sphereRobots = new ArrayList<>();
      List<Robot> robots = new ArrayList<>();
      List<YoGraphicsListRegistry> yoGraphicsListRegistries = new ArrayList<>();
      List<SimplePusherController> pusherControllers = new ArrayList<>();

      for (int i = 0; i < NumRobots; i++)
      {
         yoGraphicsListRegistries.add(new YoGraphicsListRegistry());
         initialPositions.add(new Vector3D(0.0, NextRobotOffset * i, nominalHeight));

         sphereRobots.add(new SimpleSphereRobot(i,
                                                "SphereRobot" + Integer.toString(i + 1),
                                                gravity,
                                                controlDT,
                                                nominalHeight,
                                                yoGraphicsListRegistries.get(i)));
         sphereRobots.get(i).initRobot(initialPositions.get(i), new Vector3D());
         robots.add(sphereRobots.get(i).getScsRobot());

         dcmPlans.add(new SimpleBipedCoMTrajectoryPlanner(soleFrames,
                                                          gravity,
                                                          nominalHeight,
                                                          sphereRobots.get(i).getOmega0Provider(),
                                                          sphereRobots.get(i).getScsRobot().getRobotsYoVariableRegistry(),
                                                          yoGraphicsListRegistries.get(i)));
         //This step actually add the footstep plan to the BipedPlanner
         AddCSPStepsToDCMPlan(dcmPlans.get(i), initialPositions.get(i));
         AddFootstepAndTimingStepsToDCMPlan(dcmPlans.get(i), initialPositions.get(i));
         
         switch (i)
         {
            case 1:
               sphereControllers.add(new SimpleBasicSphereController(sphereRobots.get(i), dcmPlans.get(i), yoGraphicsListRegistries.get(i)));
               break;
            case 0:
               sphereControllers.add(new SimpleLQRSphereController(sphereRobots.get(i), dcmPlans.get(i), yoGraphicsListRegistries.get(i)));
               break;
         }
         sphereControllers.get(i).solveForTrajectory();

         pusherControllers.add(createPusher(sphereRobots.get(i), yoGraphicsListRegistries.get(i)));
         setupGroundContactModel(sphereRobots.get(i).getScsRobot());
      }

      Robot[] robotArray = robots.toArray(new Robot[robots.size()]);
      
      
      
      //Create Simulatiion Construction Set
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(160000);
      scs = new SimulationConstructionSet(robotArray, parameters);

      // Create Plotter Factory
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.setVariableNameToTrack("centerOfMass");
      for (int i = 0; i < NumRobots; i++)
      {
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistries.get(i));
      }
      plotterFactory.createOverheadPlotter();

      //Create Push Button
      JButton button = new JButton("PushRobot");
      button.setToolTipText("Click to push the robot as defined in the variables 'pushDirection' and 'pushMagnitude'");
      for (int i = 0; i < NumRobots; i++)
      {
         pusherControllers.get(i).bindSCSPushButton(button);
      }
      scs.addButton(button);

      // Define simulation Parameters and run
      scs.setDT(controlDT, 1);
      scs.setCameraPosition(-3.0, -5.0, 2.0);
      scs.setCameraFix(0.0, 0.0, 0.4);
      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);
      
      scs.setupGraph("t");
      scs.setSimulateDuration(10);
      
      scs.startOnAThread();
      
      //Also flight and swing stuff
      //YoDouble omega = new YoDouble("omega", registry);
      //omega.set(Math.sqrt(gravity / nominalHeight));
   }

   //Create SoleFrames to feed into the Planner
   private static SideDependentList<MovingReferenceFrame> createSoleFrames()
   {
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         TranslationMovingReferenceFrame soleFrame = new TranslationMovingReferenceFrame(robotSide + "SoleFrame", worldFrame);
         Vector3D translation = new Vector3D();
         translation.setY(robotSide.negateIfRightSide(stepWidth / 2));
         soleFrame.updateTranslation(translation);

         soleFrames.put(robotSide, soleFrame);
      }

      return soleFrames;
   }

   public SideDependentList<MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   public Graphics3DObject getWorldGraphics()
   {
      return worldGraphics;
   }
   
   // Add ability to push the robot during the simulation
   private static SimplePusherController createPusher(SimpleSphereRobot sphereRobot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Joint joint = sphereRobot.getScsRobot().getJoint(sphereRobot.getRootJoint().getName());
      SimplePusherController pushController = new SimplePusherController(sphereRobot.getScsRobot(), joint, new Vector3D(), 0.05);

      pushController.setPushForceMagnitude(10.0);
      pushController.setPushDuration(0.25);
      pushController.setPushForceDirection(new Vector3D(0.0, 1.0, 0.0));
      yoGraphicsListRegistry.registerYoGraphic(sphereRobot.getScsRobot().getName() + "Pusher", pushController.getForceVisualizer());

      return pushController;
   }

   // Implement a ground contact model
   private static void setupGroundContactModel(Robot robot)
   {
      double kXY = 1000.0; //1422.0;
      double bXY = 100.0; //150.6;
      double kZ = 20.0; //50.0;
      double bZ = 50.0; //1000.0;
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry());

      GroundProfile3D groundProfile = new FlatGroundProfile();
      groundContactModel.setGroundProfile3D(groundProfile);
      robot.setGroundContactModel(groundContactModel);
   }

   // Define the contact sequence for the robot as a List of Contact State Providers

   private static void AddCSPStepsToDCMPlan(SimpleBipedCoMTrajectoryPlanner dcmPlan, Vector3DReadOnly shift)
   {
      double contactX = shift.getX();
      double contactY = shift.getY();
      double width = stepWidth;
      dcmPlan.clearInputStepSequence();

      //Create starting contact: COP moves from origin to first footstep location
      SettableContactStateProvider initialContactStateProvider = new SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, initialTransferDuration);
      initialContactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
      contactY += width / 2;
      initialContactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT); //contact or flight
      dcmPlan.addStepToSequence(initialContactStateProvider);
      double currentTime = initialTransferDuration;

      //Create steps, COP starts at one step location and moves to the next
      for (int i = 0; i < numberOfSteps; i++)
      {
         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();

         contactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
         contactX += stepLength;
         width = -width;
         contactY += width;
         contactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         dcmPlan.addStepToSequence(contactStateProvider);

         currentTime += stepDuration;
      }

      SettableContactStateProvider finalStateProvider = new SettableContactStateProvider();
      finalStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
      finalStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactX, shift.getY(), 0.0));
      finalStateProvider.getTimeInterval().setInterval(currentTime, currentTime + finalTransferDuration);
      finalStateProvider.setContactState(ContactState.IN_CONTACT);

      dcmPlan.addStepToSequence(finalStateProvider);

   }

   private final static double swingDurationShiftFraction = 0.5;
   private final static double swingSplitFraction = 0.5;
   private final static double transferSplitFraction = 0.5;
   private final static double transferWeightDistribution = 0.5;

   
   /*
    * Footsteps contain information by the position of a planned footstep. The RobotSide of the footstep is the 
    * side that will swing to touch down at the planned position. The footstep timing begins at the lift_off of
    * this swing foot. The swing takes swingDuration time, then the double support takes transitionDuration where
    * the COP moves from previous foot to new (formerly swinging) foot.
    */
   private static void AddFootstepAndTimingStepsToDCMPlan(SimpleBipedCoMTrajectoryPlanner dcmPlan, Vector3DReadOnly shift)
   {
      FootstepShiftFractions newShiftFractions = 
            new FootstepShiftFractions(swingDurationShiftFraction,swingSplitFraction,
                                       transferSplitFraction,transferWeightDistribution);
      double contactX = shift.getX();
      double contactY = shift.getY();
      double width = stepWidth;
      double stepStartTime = initialTransferDuration;
      RobotSide currentSide = RobotSide.LEFT;
      dcmPlan.clearConvertedStepSequence();
      Quaternion unitQuaternion = new Quaternion(0, 0, 0, 1);
      //Initial Position
      //Footstep InitStep = GenerateFootstep(currentSide, new Point3D(contactX,contactY,0), new Quaternion(0, 0, 0, 1));
      //dcmPlan.addStepToSequence(InitStep, new FootstepTiming(initialTransferDuration, 0), newShiftFractions, 0);
      contactY += stepWidth / 2; //begin planning ctnctY at pos of left foot

      //Main Steps
      for (int i = 0; i < numberOfSteps; i++)
      {
         contactX += stepLength;
         width = -width;
         contactY += width;
         currentSide = currentSide.getOppositeSide();

         Footstep newStep = new Footstep();
         newStep.setRobotSide(currentSide);
         newStep.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(contactX, contactY, 0), unitQuaternion));
         
         FootstepTiming newTiming = new FootstepTiming(swingDuration, stanceDuration);
         newTiming.setAbsoluteTime(0, stepStartTime);
         
         dcmPlan.addStepToSequence(newStep, newTiming, newShiftFractions, 0);
         stepStartTime += newTiming.getStepTime();
      }
      
      //Final Position
      Footstep finStep = new Footstep();
      finStep.setRobotSide(currentSide);
      finStep.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(contactX, 0, 0), unitQuaternion));
      
      FootstepTiming finTiming = new FootstepTiming(swingDuration, finalTransferDuration);
      finTiming.setAbsoluteTime(0, stepStartTime);
      
      dcmPlan.addStepToSequence(finStep, finTiming, newShiftFractions, 0);
   }

   public static void main(String[] args)
   {
      SimpleLQRMomentumControllerSimulation simulation = new SimpleLQRMomentumControllerSimulation();
   }
}
