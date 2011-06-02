package us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl;

import java.util.EnumMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointAccelerationCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpper;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;

public class CraigPage300SwingLegTorqueControlModule implements SwingLegTorqueControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PDPlusIDSwingLegTorqueControlModule");
   private final LegJointName[] legJointNames;

   private final ProcessedSensorsInterface processedSensors;

   private final CommonWalkingReferenceFrames referenceFrames;
   private final FullRobotModel fullRobotModel;
   private final CouplingRegistry couplingRegistry;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredYoLegJointPositions =
      SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredYoLegJointVelocities =
      SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> jointPositionErrors = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> jointVelocityErrors = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> kpGains = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> kdGains = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   
   private final DoubleYoVariable softScaleFactor = new DoubleYoVariable("softScaleFactor", registry);
   private final DoubleYoVariable ankleTorqueScale = new DoubleYoVariable("ankleTorqueScale", registry);

   private final SideDependentList<LegJointPositions> desiredLegJointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredLegJointVelocities = new SideDependentList<LegJointVelocities>();

   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;
   private final SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators;
   private final SideDependentList<DesiredJointAccelerationCalculator> desiredJointAccelerationCalculators;

   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators;

   private final BooleanYoVariable inverseKinematicsExceptionHasBeenThrown = new BooleanYoVariable("kinematicException", registry);
   private final DoubleYoVariable jacobianDeterminant = new DoubleYoVariable("jacobianDeterminant", registry);
   private boolean useBodyAcceleration;


   public CraigPage300SwingLegTorqueControlModule(LegJointName[] legJointNames, ProcessedSensorsInterface processedSensors,
           CommonWalkingReferenceFrames referenceFrames, FullRobotModel fullRobotModel, CouplingRegistry couplingRegistry,
           LegInverseKinematicsCalculator inverseKinematicsCalculator, SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators,
           SideDependentList<DesiredJointAccelerationCalculator> desiredJointAccelerationCalculators,
           SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators, YoVariableRegistry parentRegistry, GUISetterUpperRegistry guiSetterUpperRegistry)
   {
      this.legJointNames = legJointNames;
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.couplingRegistry = couplingRegistry;
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.desiredJointVelocityCalculators = desiredJointVelocityCalculators;
      this.desiredJointAccelerationCalculators = desiredJointAccelerationCalculators;
      this.inverseDynamicsCalculators = inverseDynamicsCalculators;

      for (RobotSide robotSide : RobotSide.values())
      {
         this.desiredLegJointPositions.put(robotSide, new LegJointPositions(robotSide));
         this.desiredLegJointVelocities.put(robotSide, new LegJointVelocities(legJointNames, robotSide));

         for (LegJointName legJointName : legJointNames)
         {
            String jointName = robotSide.getCamelCaseNameForStartOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression();
            desiredYoLegJointPositions.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "DesiredPosition", parentRegistry));
            desiredYoLegJointVelocities.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "DesiredVelocity", parentRegistry));
            jointPositionErrors.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "PositionError", parentRegistry));
            jointVelocityErrors.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "VelocityError", parentRegistry));
         }
      }
      
      for (LegJointName legJointName : legJointNames)
      {
         String jointName = legJointName.getCamelCaseNameForMiddleOfExpression();
         kpGains.put(legJointName, new DoubleYoVariable(jointName + "KpGain", parentRegistry));
         kdGains.put(legJointName, new DoubleYoVariable(jointName + "KdGain", parentRegistry));
      }

      parentRegistry.addChild(registry);
      guiSetterUpperRegistry.registerGUISetterUpper(createGUISetterUpper());
   }

   public void compute(LegTorques legTorquesToPackForSwingLeg, FramePoint desiredFootPosition, Orientation desiredFootOrientation,
                       FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity, FrameVector desiredFootAcceleration,
                       FrameVector desiredFootAngularAcceleration)
   {
      // robotSides
      RobotSide swingSide = legTorquesToPackForSwingLeg.getRobotSide();

      // reference frames
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(swingSide);
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      // Desired positions
      Transform3D footToPelvis = computeDesiredTransform(pelvisFrame, desiredFootPosition, desiredFootOrientation);
      Twist desiredTwistOfSwingFootWithRespectToWorld = computeDesiredTwist(worldFrame, footFrame, desiredFootVelocity, desiredFootAngularVelocity);

      Matrix3d footToPelvisOrientation = new Matrix3d();
      footToPelvis.get(footToPelvisOrientation);
      double desiredHipYaw = RotationFunctions.getYaw(footToPelvisOrientation);    // TODO: wrong and not necessary for R2, but ok for now.
      try
      {
         inverseKinematicsCalculator.solve(desiredLegJointPositions.get(swingSide), footToPelvis, swingSide, desiredHipYaw);
         inverseKinematicsExceptionHasBeenThrown.set(false);
      }
      catch (InverseKinematicsException e)
      {
         inverseKinematicsExceptionHasBeenThrown.set(true);
      }

      // Desired velocities
      DesiredJointVelocityCalculator desiredJointVelocityCalculator = desiredJointVelocityCalculators.get(swingSide);
      desiredJointVelocityCalculator.packDesiredJointVelocities(desiredLegJointVelocities.get(swingSide), desiredTwistOfSwingFootWithRespectToWorld);

      // set body acceleration
      if (useBodyAcceleration)
      {
         SpatialAccelerationVector bodyAcceleration = processedSensors.computeAccelerationOfPelvisWithRespectToWorld();    // FIXME: set to LIPM-based predicted body acceleration
         bodyAcceleration.setAngularPart(new Vector3d());    // zero desired angular acceleration
         bodyAcceleration.setLinearPart(new Vector3d());    // zero linear acceleration as well for now
         fullRobotModel.getRootJoint().setDesiredAcceleration(bodyAcceleration);
      }

      // Desired acceleration
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = computeDesiredSwingFootSpatialAcceleration(elevatorFrame, footFrame,
                                                                                      desiredFootAcceleration, desiredFootAngularAcceleration);
      jacobianDeterminant.set(desiredJointVelocityCalculator.swingFullLegJacobianDeterminant());
      desiredJointAccelerationCalculators.get(swingSide).compute(desiredAccelerationOfSwingFootWithRespectToWorld);

      double percentScaling = getPercentScalingBasedOnJacobianDeterminant(jacobianDeterminant.getDoubleValue());

      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      for (LegJointName legJointName : legJointNames)
      {
         RevoluteJoint revoluteJoint = fullRobotModel.getLegJoint(swingSide, legJointName);
         double qddDesired = revoluteJoint.getQddDesired();
         revoluteJoint.setQddDesired(qddDesired * percentScaling);

         double desiredJointPosition = desiredLegJointPositions.get(swingSide).getJointPosition(legJointName);
         desiredYoLegJointPositions.get(swingSide).get(legJointName).set(desiredJointPosition);
         double positionError = desiredJointPosition - processedSensors.getLegJointPosition(swingSide, legJointName);
         jointPositionErrors.get(swingSide).get(legJointName).set(positionError);

         double desiredJointVelocity = desiredLegJointVelocities.get(swingSide).getJointVelocity(legJointName);
         desiredYoLegJointVelocities.get(swingSide).get(legJointName).set(desiredJointVelocity);
         double velocityError = desiredJointVelocity - processedSensors.getLegJointVelocity(swingSide, legJointName);
         jointVelocityErrors.get(swingSide).get(legJointName).set(velocityError);

         double kpGain = kpGains.get(legJointName).getDoubleValue();
         double kdGain = kdGains.get(legJointName).getDoubleValue();
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(positionError * kpGain + velocityError * kdGain + qddDesired);
      }

      // control
      inverseDynamicsCalculators.get(swingSide).compute();

      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         double tauInverseDynamics = fullRobotModel.getLegJoint(swingSide, legJointName).getTau();

         boolean isAnAnkleJoint = legJointName == LegJointName.ANKLE_PITCH || legJointName == LegJointName.ANKLE_ROLL;
         if (isAnAnkleJoint)
            tauInverseDynamics *= ankleTorqueScale.getDoubleValue();
         
         legTorquesToPackForSwingLeg.addTorque(legJointName, tauInverseDynamics);
      }

      setUpperBodyWrench();
   }

   public void computePreSwing(RobotSide swingSide)
   {
      fullRobotModel.getRootJoint().setDesiredAccelerationToZero();

      for (LegJointName legJointName : legJointNames)
      {
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(0.0);
      }

      inverseDynamicsCalculators.get(swingSide).compute();
      setUpperBodyWrench();
   }

   public void setAnkleGainsSoft(RobotSide swingSide)
   {
      ankleTorqueScale.set(softScaleFactor.getDoubleValue());
   }

   public void setAnkleGainsDefault(RobotSide swingSide)
   {
      ankleTorqueScale.set(1.0);
   }

   private Transform3D computeDesiredTransform(ReferenceFrame pelvisFrame, FramePoint desiredFootPosition, Orientation desiredFootOrientation)
   {
      desiredFootOrientation.changeFrame(pelvisFrame);
      desiredFootPosition.changeFrame(pelvisFrame);
      Transform3D footToPelvis = createTransform(desiredFootOrientation, desiredFootPosition);

      return footToPelvis;
   }

   private Twist computeDesiredTwist(ReferenceFrame worldFrame, ReferenceFrame footFrame, FrameVector desiredFootVelocity,
                                     FrameVector desiredFootAngularVelocity)
   {
      desiredFootVelocity.changeFrame(footFrame);
      desiredFootAngularVelocity.changeFrame(footFrame);
      Twist desiredTwistOfSwingFootWithRespectToStanceFoot = new Twist(footFrame, worldFrame, footFrame, desiredFootVelocity.getVector(),
                                                                desiredFootAngularVelocity.getVector());

      return desiredTwistOfSwingFootWithRespectToStanceFoot;
   }

   private SpatialAccelerationVector computeDesiredSwingFootSpatialAcceleration(ReferenceFrame elevatorFrame, ReferenceFrame footFrame,
           FrameVector desiredSwingFootAcceleration, FrameVector desiredSwingFootAngularAcceleration)
   {
      desiredSwingFootAcceleration.changeFrame(footFrame);
      desiredSwingFootAngularAcceleration.changeFrame(footFrame);
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame,
                                                                                      desiredSwingFootAcceleration.getVector(),
                                                                                      desiredSwingFootAngularAcceleration.getVector());

      return desiredAccelerationOfSwingFootWithRespectToWorld;
   }

   private static Transform3D createTransform(Orientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3d();
      Transform3D ret = new Transform3D(rotationMatrix, new Vector3d(framePoint.getPoint()), 1.0);

      return ret;
   }

   private double getPercentScalingBasedOnJacobianDeterminant(double jacobianDeterminant)
   {
      double determinantThresholdOne = 0.06;    // 0.05;    // 0.025;
      double determinantThresholdTwo = 0.03;    // 0.02; //0.01;

      double percent = (Math.abs(jacobianDeterminant) - determinantThresholdTwo) / (determinantThresholdOne - determinantThresholdTwo);
      percent = MathTools.clipToMinMax(percent, 0.0, 1.0);

      return percent;
   }

   private void setUpperBodyWrench()
   {
      Wrench upperBodyWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(upperBodyWrench);
      upperBodyWrench.changeBodyFrameAttachedToSameBody(referenceFrames.getPelvisFrame());
      upperBodyWrench.changeFrame(referenceFrames.getPelvisFrame());
      couplingRegistry.setUpperBodyWrench(upperBodyWrench);
   }

   public void setParametersForR2()
   {
      useBodyAcceleration = false;

      for (LegJointName legJointName : legJointNames)
      {
         kpGains.get(legJointName).set(1000.0);
         kdGains.get(legJointName).set(50.0);
      }
      softScaleFactor.set(0.25);
   }

   public void setParametersForM2V2()
   {
      useBodyAcceleration = true;
      
      for (LegJointName legJointName : legJointNames)
      {
         kpGains.get(legJointName).set(600.0);
         kdGains.get(legJointName).set(10.0);
      }
      softScaleFactor.set(0.25);
   }
   
   private GUISetterUpper createGUISetterUpper()
   {
      GUISetterUpper ret = new GUISetterUpper()
      {
         public void setupGUI(SimulationConstructionSet scs)
         {
            int numberOfLegJointNames = legJointNames.length;

            String[][][] positionGraphGroupStrings = new String[numberOfLegJointNames][][];
            String[][][] velocityGraphGroupStrings = new String[numberOfLegJointNames][][];

            String[] entryBoxGroupStrings = new String[2 * numberOfLegJointNames];    // times two, because we want both kp and kd

            for (RobotSide robotSide : RobotSide.values())
            {
               for (int jointNameIndex = 0; jointNameIndex < numberOfLegJointNames; jointNameIndex++)
               {
                  LegJointName jointName = legJointNames[jointNameIndex];

                  // positions
                  String desiredPositionName = desiredYoLegJointPositions.get(robotSide).get(jointName).getName();
                  String actualPositionName = processedSensors.getLegJointPositionName(robotSide, jointName);

                  positionGraphGroupStrings[jointNameIndex] = new String[][]
                                                                           {
                        {desiredPositionName, actualPositionName}, {"auto"}
                                                                           };

                  // velocities
                  String desiredVelocityName = desiredYoLegJointVelocities.get(robotSide).get(jointName).getName();
                  String actualVelocityName = processedSensors.getLegJointVelocityName(robotSide, jointName);

                  velocityGraphGroupStrings[jointNameIndex] = new String[][]
                                                                           {
                        {desiredVelocityName, actualVelocityName}, {"auto"}
                                                                           };

                  // kp, kd
                  entryBoxGroupStrings[jointNameIndex] = kpGains.get(jointName).getName();
                  entryBoxGroupStrings[numberOfLegJointNames + jointNameIndex] = kdGains.get(jointName).getName();
               }

               String sideString = "(" + robotSide.getCamelCaseNameForStartOfExpression() + ")";
               scs.setupGraphGroup("CraigPage300 - positions " + sideString, positionGraphGroupStrings, 2);
               scs.setupGraphGroup("CraigPage300 - velocities " + sideString, velocityGraphGroupStrings, 2);
               scs.setupEntryBoxGroup("CraigPage300", entryBoxGroupStrings);
               scs.setupConfiguration("CraigPage300", "all", "CraigPage300 - velocities", "CraigPage300");
            }
         }
      };
      return ret;
   }

}
