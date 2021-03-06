package us.ihmc.exampleSimulations.skippy;

import java.util.EnumMap;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class SkippyRobotV2 extends Robot
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame footReferenceFrame;
   private final ReferenceFrame leftShoulderFrame;
   private final ReferenceFrame rightShoulderFrame;

   private final RigidBodyBasics elevator;
   private final SixDoFJoint rootJoint;

   private static final boolean SHOW_MASS_ELIPSOIDS = false;
   private static final boolean SHOW_COORDINATE_SYSTEMS = true;

   private static final AppearanceDefinition SHOULDER_COLOR = YoAppearance.Red();
   private static final AppearanceDefinition TORSO_COLOR = YoAppearance.Blue();
   private static final AppearanceDefinition LEG_COLOR = YoAppearance.Blue();
   private static final AppearanceDefinition JOINT_COLOR = YoAppearance.LightSteelBlue();
   private static final double JOINT_RADIUS = 0.1;

   public static final double LEG_LENGTH = 1.0;
   public static final double LEG_MASS = 1.5;
   public static final double LEG_RADIUS = 0.05;

   public static final double TORSO_LENGTH = 2.0;
   public static final double TORSO_MASS = 1.0;
   public static final double TORSO_RADIUS = 0.05;

   public static final double SHOULDER_LENGTH = 3.0;
   public static final double SHOULDER_MASS = 0.5;
   public static final double SHOULDER_RADIUS = 0.05;

   private final GroundContactPoint footContactPoint;
   private ExternalForcePoint rootJointForce;


   private enum SkippyJoint
   {
      HIP_PITCH, SHOULDER_ROLL;
      public static SkippyJoint[] values = {HIP_PITCH, SHOULDER_ROLL};

   }

   private enum SkippyBody
   {
      ELEVATOR, LEG, TORSO, SHOULDER
   }

   private final EnumMap<SkippyJoint, OneDoFJointBasics> jointMap = new EnumMap<>(SkippyJoint.class);
   private final EnumMap<SkippyJoint, PinJoint> scsJointMap = new EnumMap<>(SkippyJoint.class);
   private final EnumMap<SkippyBody, RigidBodyBasics> bodyMap = new EnumMap<>(SkippyBody.class);
   private final FloatingJoint scsRootJoint;

   public SkippyRobotV2()
   {
      super("SkippyV2");

      // --- id robot ---
      elevator = new RigidBody("elevator", worldFrame);
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      bodyMap.put(SkippyBody.ELEVATOR, elevator);

      rootJoint = new SixDoFJoint("rootJoint", elevator);
      Matrix3D inertiaTorso = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(TORSO_MASS, TORSO_RADIUS, TORSO_LENGTH, Axis3D.Z);
      RigidBodyBasics torso = new RigidBody("torso", rootJoint, inertiaTorso, TORSO_MASS, new Vector3D(0.0, 0.0, 0.0));
      bodyMap.put(SkippyBody.TORSO, torso);

      RevoluteJoint idHipJoint = new RevoluteJoint("idHipJoint", torso, new Vector3D(0.0, 0.0, -TORSO_LENGTH / 2.0), new Vector3D(1.0, 0.0, 0.0));
      Matrix3D inertiaLeg = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(LEG_MASS, LEG_RADIUS, LEG_LENGTH, Axis3D.Z);
      RigidBodyBasics leg = new RigidBody("leg", idHipJoint, inertiaLeg, LEG_MASS, new Vector3D(0.0, 0.0, -LEG_LENGTH / 2.0));
      bodyMap.put(SkippyBody.LEG, leg);
      jointMap.put(SkippyJoint.HIP_PITCH, idHipJoint);

      RigidBodyTransform legToFoot = new RigidBodyTransform();
      legToFoot.getTranslation().set(0.0, 0.0, -LEG_LENGTH / 2.0);
      footReferenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("footFrame", leg.getBodyFixedFrame(), legToFoot);

      RevoluteJoint idShoulderJoint = new RevoluteJoint("idShoulderJoint", torso, new Vector3D(0.0, 0.0, TORSO_LENGTH / 2.0), new Vector3D(0.0, 1.0, 0.0));
      Matrix3D inertiaShoulder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(SHOULDER_MASS, SHOULDER_RADIUS, SHOULDER_LENGTH, Axis3D.X);
      RigidBodyBasics shoulder = new RigidBody("shoulder", idShoulderJoint, inertiaShoulder, SHOULDER_MASS, new Vector3D(0.0, 0.0, 0.0));
      jointMap.put(SkippyJoint.SHOULDER_ROLL, idShoulderJoint);
      bodyMap.put(SkippyBody.SHOULDER, shoulder);

      //One end effector at each shoulder extremity
      RigidBodyTransform leftShoulder = new RigidBodyTransform();
      leftShoulder.getTranslation().set(-SHOULDER_LENGTH / 2.0, 0.0, 0.0);
      leftShoulderFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("leftShoulderFrame", shoulder.getBodyFixedFrame(), leftShoulder);

      RigidBodyTransform rightShoulder = new RigidBodyTransform();
      rightShoulder.getTranslation().set(SHOULDER_LENGTH / 2.0, 0.0, 0.0);
      rightShoulderFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("rightShoulderFrame", shoulder.getBodyFixedFrame(), rightShoulder);

      // --- scs robot ---
      scsRootJoint = new FloatingJoint("rootJoint", new Vector3D(), this);
      scsRootJoint.setLink(createTorsoSkippy());
      scsRootJoint.setPosition(0.0, 0.0, LEG_LENGTH + TORSO_LENGTH / 2.0);
      rootJointForce = new ExternalForcePoint("rootJointForce", new Vector3D(0.0, 0.0, LEG_LENGTH + TORSO_LENGTH / 2.0), this);

      this.addRootJoint(scsRootJoint);

      PinJoint shoulderJoint = new PinJoint("shoulderJoint", new Vector3D(0.0, 0.0, TORSO_LENGTH / 2.0), this, Axis3D.Y);
      shoulderJoint.setLink(createArm());
      scsRootJoint.addJoint(shoulderJoint);
      scsJointMap.put(SkippyJoint.SHOULDER_ROLL, shoulderJoint);

      PinJoint hipJoint = new PinJoint("hip", new Vector3D(0.0, 0.0, -TORSO_LENGTH / 2.0), this, Axis3D.X);
      hipJoint.setLink(createLeg());
      scsRootJoint.addJoint(hipJoint);
      scsJointMap.put(SkippyJoint.HIP_PITCH, hipJoint);

      // add ground contact points
      footContactPoint = new GroundContactPoint("gc_foot", new Vector3D(0.0, 0.0, -LEG_LENGTH), this);
      hipJoint.addGroundContactPoint(footContactPoint);
      GroundContactPoint hipContactPoint = new GroundContactPoint("gc_hip", new Vector3D(0.0, 0.0, 0.0), this);
      hipJoint.addGroundContactPoint(hipContactPoint);
      GroundContactPoint arm1ContactPoint = new GroundContactPoint("gc_arm1", new Vector3D(SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      shoulderJoint.addGroundContactPoint(arm1ContactPoint);
      GroundContactPoint arm2ContactPoint = new GroundContactPoint("gc_arm2", new Vector3D(-SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      shoulderJoint.addGroundContactPoint(arm2ContactPoint);

      // add ground contact model
      LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoRegistry());
      ground.setZStiffness(2000.0);
      ground.setZDamping(1500.0);
      ground.setXYStiffness(50000.0);
      ground.setXYDamping(2000.0);
      ground.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(ground);

      // add an external force point to easily push the robot in simulation
      ExternalForcePoint forcePoint = new ExternalForcePoint("forcePoint", new Vector3D(0.0, 0.0, TORSO_LENGTH / 2.0), this);
      scsRootJoint.addExternalForcePoint(forcePoint);
   }

   private Link createTorsoSkippy()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, 0.0);
      torso.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(TORSO_MASS, TORSO_RADIUS, TORSO_LENGTH, Axis3D.Z));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -TORSO_LENGTH / 2.0);
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, TORSO_COLOR);
      torso.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         torso.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         torso.addCoordinateSystemToCOM(0.3);

      return torso;
   }

   private Link createArm()
   {
      Link arms = new Link("arms");
      arms.setMass(SHOULDER_MASS);
      arms.setComOffset(0.0, 0.0, 0.0);
      arms.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(SHOULDER_MASS, SHOULDER_RADIUS, SHOULDER_LENGTH, Axis3D.X));

      // arm graphics:
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis3D.Y);
      linkGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH / 2.0);
      linkGraphics.addCylinder(SHOULDER_LENGTH, SHOULDER_RADIUS, SHOULDER_COLOR);
      // joint graphics:
      linkGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      linkGraphics.rotate(Math.PI / 2.0, Axis3D.X);
      linkGraphics.translate(-SHOULDER_LENGTH / 2.0, 0.0, -JOINT_RADIUS);
      linkGraphics.addCylinder(2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS / 3.0, JOINT_COLOR);
      arms.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         arms.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         arms.addCoordinateSystemToCOM(0.3);

      return arms;
   }

   private Link createLeg()
   {
      Link leg = new Link("leg");
      leg.setMass(LEG_MASS);
      leg.setComOffset(0.0, 0.0, -LEG_LENGTH / 2.0);
      leg.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(LEG_MASS, LEG_RADIUS, LEG_LENGTH, Axis3D.Z));

      // leg graphics:
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -LEG_LENGTH);
      linkGraphics.addCylinder(LEG_LENGTH, LEG_RADIUS, LEG_COLOR);
      // joint graphics:
      linkGraphics.identity();
      linkGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      linkGraphics.translate(0.0, 0.0, -JOINT_RADIUS);
      linkGraphics.addCylinder(2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS / 3.0, JOINT_COLOR);
      leg.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         leg.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         leg.addCoordinateSystemToCOM(0.3);

      return leg;
   }

   public void updateInverseDynamicsStructureFromSimulation()
   {
      // update joint angles and velocities
      for (SkippyJoint joint : SkippyJoint.values)
      {
         OneDoFJointBasics idJoint = jointMap.get(joint);
         OneDegreeOfFreedomJoint scsJoint = scsJointMap.get(joint);
         idJoint.setQ(scsJoint.getQYoVariable().getDoubleValue());
         idJoint.setQd(scsJoint.getQDYoVariable().getDoubleValue());
      }

      // update root joint position
      RigidBodyTransform rootJointTransform = new RigidBodyTransform();
      scsRootJoint.getTransformToWorld(rootJointTransform);
      rootJointTransform.getRotation().normalize();
      rootJoint.setJointConfiguration(rootJointTransform);

      // update root joint velocity
      FrameVector3D linearVelocity = new FrameVector3D();
      FrameVector3D angularVelocity = new FrameVector3D();
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame rootBodyFrame = rootJoint.getFrameAfterJoint();
      scsRootJoint.getVelocity(linearVelocity);
      linearVelocity.changeFrame(rootBodyFrame);
      scsRootJoint.getAngularVelocity(angularVelocity, rootBodyFrame);
      Twist rootJointTwist = new Twist(rootBodyFrame, elevatorFrame, rootBodyFrame, angularVelocity, linearVelocity);
      rootJoint.setJointTwist(rootJointTwist);

      // update all the frames
      bodyMap.get(SkippyBody.ELEVATOR).updateFramesRecursively();
   }

   public void updateSimulationFromInverseDynamicsTorques()
   {
      for (SkippyJoint joint : SkippyJoint.values())
      {
         OneDoFJointBasics idJoint = jointMap.get(joint);
         OneDegreeOfFreedomJoint scsJoint = scsJointMap.get(joint);
         scsJoint.setTau(idJoint.getTau());
      }
   }

   public Point3D getFootLocation()
   {
      return footContactPoint.getPositionCopy();
   }

   public RigidBodyBasics getSkippyFoot()
   {
      return bodyMap.get(SkippyBody.LEG);
   }

   public Point3D computeFootLocation()
   {
      return footContactPoint.getPositionCopy();
   }

   public boolean getFootFS()
   {
      return footContactPoint.isInContact();
   }

   public void computeFootContactForce(Vector3DBasics actualReaction)
   {
      footContactPoint.getForce(actualReaction);
   }

   public double getGravity()
   {
      return this.getGravityZ();
   }

   public ReferenceFrame getFootFrame()
   {
      return footReferenceFrame;
   }

   public RigidBodyBasics getLegBody()
   {
      return bodyMap.get(SkippyBody.LEG);
   }

   public RigidBodyBasics getShoulderBody()
   {
      return bodyMap.get(SkippyBody.SHOULDER);
   }

   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   public RigidBodyBasics getTorso()
   {
      return bodyMap.get(SkippyBody.TORSO);
   }

   public ReferenceFrame getLeftShoulderFrame()
   {
      return leftShoulderFrame;
   }

   public ReferenceFrame getRightShoulderFrame()
   {
      return rightShoulderFrame;
   }

   public void setQ_hip(double hipAngle)
   {
    scsJointMap.get(SkippyJoint.HIP_PITCH).setQ(hipAngle);
   }
   public void setRootJointForce(double x, double y, double z)
   {
      rootJointForce.setForce(x, y, z);
   }

   public void setQ_shoulder(double shoulderAngle)
   {
      scsJointMap.get(SkippyJoint.SHOULDER_ROLL).setQ(shoulderAngle);
   }

}
