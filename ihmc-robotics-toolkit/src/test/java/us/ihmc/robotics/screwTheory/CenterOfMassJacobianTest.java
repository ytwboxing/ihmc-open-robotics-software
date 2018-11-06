package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.random.RandomGeometry;

public class CenterOfMassJacobianTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RigidBodyBasics elevator;
   private Random random;

   @Before
   public void setUp()
   {
      elevator = new RigidBody("elevator", worldFrame);
      random = new Random(1986L);
   }

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSingleJoint()
   {
      ArrayList<RevoluteJoint> joints = setUpSingleJoint();
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianTwoJointsSimple()
   {
      ArrayList<RevoluteJoint> joints = setUpTwoJointsSimple();
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianRandomChain()
   {
      ArrayList<RevoluteJoint> joints = setUpRandomChain(elevator);
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTree()
   {
      ArrayList<RevoluteJoint> joints = setUpRandomTree(elevator);
      testComputeJacobianRevoluteJoints(joints, ScrewTools.computeSupportAndSubtreeSuccessors(elevator), elevator.getBodyFixedFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRigidBodyListSortInvariant()
   {
      ArrayList<RevoluteJoint> joints = setUpRandomTree(elevator);
      RigidBodyBasics[] rigidBodiesInOrder = ScrewTools.computeSupportAndSubtreeSuccessors(elevator);
      List<RigidBodyBasics> rigidBodiesInOrderArrayList = Arrays.asList(rigidBodiesInOrder);

      Collections.shuffle(rigidBodiesInOrderArrayList, random);
      RigidBodyBasics[] rigidBodiesOutOrder = rigidBodiesInOrderArrayList.toArray(new RigidBodyBasics[rigidBodiesInOrder.length]);

      ScrewTestTools.setRandomPositions(joints, random);
      elevator.updateFramesRecursively();
      ScrewTestTools.setRandomVelocities(joints, random);

      CenterOfMassJacobian jacobianInOrder = new CenterOfMassJacobian(rigidBodiesInOrder, elevator.getBodyFixedFrame());
      jacobianInOrder.compute();
      FrameVector3D velocityFromJacobianInOrder = new FrameVector3D(ReferenceFrame.getWorldFrame());
      jacobianInOrder.getCenterOfMassVelocity(velocityFromJacobianInOrder);

      CenterOfMassJacobian jacobianOutOrder = new CenterOfMassJacobian(rigidBodiesOutOrder, elevator.getBodyFixedFrame());
      jacobianOutOrder.compute();
      FrameVector3D velocityFromJacobianOutOrder = new FrameVector3D(ReferenceFrame.getWorldFrame());
      jacobianOutOrder.getCenterOfMassVelocity(velocityFromJacobianOutOrder);

      EuclidCoreTestTools.assertTuple3DEquals(velocityFromJacobianInOrder, velocityFromJacobianOutOrder, 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSixDoFPlusRandomChain()
   {
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator);
      RigidBodyBasics floatingBody = ScrewTools.addRigidBody("floating", sixDoFJoint, new Matrix3D(), random.nextDouble(), RandomGeometry.nextVector3D(random));
      ArrayList<RevoluteJoint> revoluteJoints = setUpRandomChain(floatingBody);

      CenterOfMassJacobian jacobian = new CenterOfMassJacobian(elevator);

      sixDoFJoint.setJointConfiguration(EuclidCoreRandomTools.nextRigidBodyTransform(random));
      Twist sixDoFJointTwist = new Twist();
      sixDoFJoint.getJointTwist(sixDoFJointTwist);
      sixDoFJointTwist.getLinearPart().set(RandomGeometry.nextVector3D(random));
      sixDoFJointTwist.getAngularPart().set(RandomGeometry.nextVector3D(random));
      sixDoFJoint.setJointTwist(sixDoFJointTwist);

      ScrewTestTools.setRandomPositions(revoluteJoints, random);
      elevator.updateFramesRecursively();
      ScrewTestTools.setRandomVelocities(revoluteJoints, random);

      jacobian.compute();
      FrameVector3D velocityFromJacobian = new FrameVector3D(ReferenceFrame.getWorldFrame());
      jacobian.getCenterOfMassVelocity(velocityFromJacobian);

      RigidBodyBasics rootBody = elevator;
      FrameVector3D velocityNumerical = computeCenterOfMassVelocityNumerically(sixDoFJoint, revoluteJoints, rootBody,
                                         ScrewTools.computeSupportAndSubtreeSuccessors(rootBody), elevator.getBodyFixedFrame());

      RotationMatrix rotation = new RotationMatrix();
      rotation.set(sixDoFJoint.getJointPose().getOrientation());

      EuclidCoreTestTools.assertTuple3DEquals(velocityNumerical, velocityFromJacobian, 4e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSkipLeafBody()
   {
      ArrayList<RevoluteJoint> joints = setUpTwoJointsSimple();
      RigidBodyBasics[] rigidBodies = new RigidBodyBasics[] {joints.get(0).getSuccessor()};
      testComputeJacobianRevoluteJoints(joints, rigidBodies, elevator.getBodyFixedFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeJacobianSkipIntermediateBody()
   {
      ArrayList<RevoluteJoint> joints = setUpTwoJointsSimple();
      RigidBodyBasics[] rigidBodies = new RigidBodyBasics[] {joints.get(joints.size() - 1).getSuccessor()};
      testComputeJacobianRevoluteJoints(joints, rigidBodies, elevator.getBodyFixedFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeRootJointLinearVelocity()
   {
      Random random = new Random(44345L);

      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator);
      RigidBodyBasics body0 = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      ArrayList<RevoluteJoint> revoluteJoints = new ArrayList<RevoluteJoint>();
      ScrewTestTools.createRandomTreeRobot(revoluteJoints, body0, 25, random);
      
      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);
      ScrewTestTools.setRandomPositions(revoluteJoints, random);
      ScrewTestTools.setRandomVelocities(revoluteJoints, random);
      elevator.updateFramesRecursively();

      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      Twist rootJointTwist = new Twist();
      rootJoint.getJointTwist(rootJointTwist);
      FrameVector3D rootJointLinearVelocity = new FrameVector3D(rootJointFrame);
      rootJointLinearVelocity.setIncludingFrame(rootJointTwist.getLinearPart());
      
      CenterOfMassJacobian jacobianBody = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(elevator), ScrewTools.computeSubtreeJoints(body0), elevator.getBodyFixedFrame());
      jacobianBody.compute();
      FrameVector3D comVelocityBody = new FrameVector3D(worldFrame);
      jacobianBody.getCenterOfMassVelocity(comVelocityBody);
      comVelocityBody.changeFrame(rootJointFrame);
      
      CenterOfMassJacobian jacobianWorld = new CenterOfMassJacobian(elevator);
      jacobianWorld.compute();
      FrameVector3D comVelocityWorld = new FrameVector3D(worldFrame);
      jacobianWorld.getCenterOfMassVelocity(comVelocityWorld);
      comVelocityWorld.changeFrame(rootJointFrame);
      
      FrameVector3D angularVelocityBody = new FrameVector3D(rootJointFrame);
      angularVelocityBody.setIncludingFrame(rootJointTwist.getAngularPart());

      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJointFrame);
      centerOfMassCalculator.compute();
      FramePoint3D centerOfMassBody = new FramePoint3D(worldFrame);
      centerOfMassCalculator.getCenterOfMass(centerOfMassBody);
      FrameVector3D crossPart = new FrameVector3D(rootJointFrame);
      crossPart.cross(angularVelocityBody, centerOfMassBody);
      
      FrameVector3D rootJointLinearVelocityBack = new FrameVector3D(comVelocityWorld);
      rootJointLinearVelocityBack.sub(crossPart);
      rootJointLinearVelocityBack.sub(comVelocityBody);

      assertTrue(rootJointLinearVelocity.epsilonEquals(rootJointLinearVelocityBack, 1e-12));
   }
   
   public static FrameVector3D computeCenterOfMassVelocityNumerically(SixDoFJoint sixDoFJoint, ArrayList<RevoluteJoint> revoluteJoints, RigidBodyBasics rootBody,
           RigidBodyBasics[] rigidBodiesToUse, ReferenceFrame referenceFrame)
   {
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rigidBodiesToUse, referenceFrame);

      centerOfMassCalculator.compute();
      FramePoint3D centerOfMass1 = new FramePoint3D(centerOfMassCalculator.getCenterOfMass());

      double dt = 1e-8;
      if (sixDoFJoint != null)
         ScrewTestTools.integrateVelocities(sixDoFJoint, dt);
      ScrewTestTools.integrateVelocities(revoluteJoints, dt);
      rootBody.updateFramesRecursively();

      centerOfMassCalculator.compute();
      FramePoint3D centerOfMass2 = new FramePoint3D(centerOfMassCalculator.getCenterOfMass());

      FrameVector3D velocityNumerical = new FrameVector3D(centerOfMass2);
      velocityNumerical.sub(centerOfMass1);
      velocityNumerical.scale(1.0 / dt);

      return velocityNumerical;
   }

   private void testComputeJacobianRevoluteJoints(ArrayList<RevoluteJoint> joints, RigidBodyBasics[] rigidBodies, ReferenceFrame referenceFrame)
   {
      CenterOfMassJacobian jacobian = new CenterOfMassJacobian(rigidBodies, referenceFrame);

      ScrewTestTools.setRandomPositions(joints, random);
      elevator.updateFramesRecursively();
      ScrewTestTools.setRandomVelocities(joints, random);

      jacobian.compute();
      FrameVector3D velocityFromJacobian = new FrameVector3D(ReferenceFrame.getWorldFrame());
      jacobian.getCenterOfMassVelocity(velocityFromJacobian);

      RigidBodyBasics rootBody = elevator;
      FrameVector3D velocityNumerical = computeCenterOfMassVelocityNumerically(null, joints, rootBody, rigidBodies, referenceFrame);

      EuclidCoreTestTools.assertTuple3DEquals(velocityNumerical, velocityFromJacobian, 1e-5);
   }

   private ArrayList<RevoluteJoint> setUpSingleJoint()
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      joints = new ArrayList<RevoluteJoint>();
      RevoluteJoint joint = ScrewTools.addRevoluteJoint("joint", elevator, RandomGeometry.nextVector3D(random), X);
      joints.add(joint);
      ScrewTools.addRigidBody("body", joint, new Matrix3D(), random.nextDouble(), RandomGeometry.nextVector3D(random));

      return joints;
   }

   private ArrayList<RevoluteJoint> setUpTwoJointsSimple()
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      RevoluteJoint joint1 = ScrewTools.addRevoluteJoint("joint1", elevator, new Vector3D(0.0, 1.0, 0.0), X);
      joints.add(joint1);
      RigidBodyBasics body1 = ScrewTools.addRigidBody("body1", joint1, new Matrix3D(), 2.0, new Vector3D(0.0, 1.0, 0.0));
      RevoluteJoint joint2 = ScrewTools.addRevoluteJoint("joint2", body1, new Vector3D(0.0, 1.0, 0.0), X);
      joints.add(joint2);
      ScrewTools.addRigidBody("body2", joint2, new Matrix3D(), 3.0, new Vector3D(0.0, 1.0, 0.0));

      return joints;
   }

   private ArrayList<RevoluteJoint> setUpRandomChain(RigidBodyBasics rootBody)
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      Vector3D[] jointAxes =
      {
         X, X, Y, Z, X, Y, Z, Z
      };
      ScrewTestTools.createRandomChainRobot("", joints, rootBody, jointAxes, random);

      return joints;
   }

   private ArrayList<RevoluteJoint> setUpRandomTree(RigidBodyBasics elevator)
   {
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();

      Vector3D[] jointAxes1 = {X, Y, Z, Y};
      ScrewTestTools.createRandomChainRobot("chainA", joints, elevator, jointAxes1, random);

      Vector3D[] jointAxes2 = {Z, X, Y, X};
      ScrewTestTools.createRandomChainRobot("chainB", joints, elevator, jointAxes2, random);

      Vector3D[] jointAxes3 = {Y, Y, X};
      ScrewTestTools.createRandomChainRobot("chainC", joints, joints.get(joints.size() - 2).getPredecessor(), jointAxes3, random);

      return joints;
   }
}
