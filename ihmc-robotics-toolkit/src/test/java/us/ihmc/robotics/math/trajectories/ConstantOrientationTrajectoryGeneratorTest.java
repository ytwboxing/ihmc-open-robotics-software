package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class ConstantOrientationTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-10;

   private ConstantOrientationTrajectoryGenerator generator;

   private String namePrefix = "namePrefixTEST";

   private ReferenceFrame referenceFrame;
   private OrientationProvider orientationProvider;
   private static double finalTime = 10.0;
   private YoVariableRegistry parentRegistry;

   private FrameQuaternion orientation;

   @BeforeEach
   public void setUp()
   {
      referenceFrame = new ReferenceFrame("rootNameTEST", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {

         }
      };
      orientation = new FrameQuaternion(referenceFrame);
      orientationProvider = new OrientationProvider()
      {
         @Override
         public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
         {
            orientationToPack.set(orientation);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrame;
         }
      };
      parentRegistry = new YoVariableRegistry("parentRegistryTEST");
   }

   @AfterEach
   public void tearDown()
   {
      referenceFrame = null;
      orientation = null;
      orientationProvider = null;
      parentRegistry = null;
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConstructor()
   {
      try
      {
         generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
   }

	@Test
   public void testIsDone()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      generator.initialize();
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(finalTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@Test
   public void testGet()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularVelocity()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));

      angularVelocityToPack.changeFrame(referenceFrame);
      generator.getAngularVelocity(angularVelocityToPack);

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularAcceleration()
   {
      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      angularAccelerationToPack.changeFrame(referenceFrame);

      generator.getAngularAcceleration(angularAccelerationToPack);

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularData()
   {
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);
      orientationToPack.setYawPitchRollIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, parentRegistry);
      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));
      angularVelocityToPack.changeFrame(referenceFrame);

      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));
      angularAccelerationToPack.changeFrame(referenceFrame);

      generator.getAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);

      assertEquals(0.0, orientationToPack.getYaw(), EPSILON);
      assertEquals(0.0, orientationToPack.getPitch(), EPSILON);
      assertEquals(0.0, orientationToPack.getRoll(), EPSILON);
      assertSame(referenceFrame, orientationToPack.getReferenceFrame());

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }
}