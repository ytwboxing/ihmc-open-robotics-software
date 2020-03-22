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
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class OrientationInterpolationTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefixTEST";
   private FrameQuaternion orientation;

   private ReferenceFrame referenceFrame;
   private DoubleProvider trajectoryTimeProvider;
   private OrientationProvider initialOrientationProvider;
   private OrientationProvider finalOrientationProvider;
   private YoVariableRegistry parentRegistry;

//   private static int globalCounter = 0;
   private static final double EPSILON = 1e-10;
   private static double trajectoryTime = 10.0;

   private OrientationInterpolationTrajectoryGenerator generator;

   @BeforeEach
   public void setUp()
   {
      referenceFrame = new ReferenceFrame("test", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      orientation = new FrameQuaternion(referenceFrame);

      trajectoryTimeProvider = new SettableDoubleProvider(trajectoryTime);
      initialOrientationProvider = new OrientationProvider()
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
      finalOrientationProvider = new OrientationProvider()
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
      parentRegistry = new YoVariableRegistry("registry");
   }

   @AfterEach
   public void tearDown()
   {
      orientation = null;

      referenceFrame = null;
      trajectoryTimeProvider = null;
      initialOrientationProvider = null;
      finalOrientationProvider = null;
      parentRegistry = null;
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConstructor()
   {
      try
      {
         generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, null);
         fail();
      }
      catch(NullPointerException npe) {}

      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
   }

	@Test
   public void testIsDone()
   {
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      generator.initialize();
      generator.setContinuouslyUpdateFinalOrientation(true);
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(trajectoryTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@Test
   public void testGet()
   {
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularVelocity()
   {
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      FrameVector3D angularVelocityToPack = new FrameVector3D(referenceFrame, 10.0, 10.0, 10.0);

      assertTrue(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));

      generator.getAngularVelocity(angularVelocityToPack);

     assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
     assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
     assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
     assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@Test
   public void testPackAngularAcceleration()
   {
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(referenceFrame, 10.0, 10.0, 10.0);

      assertTrue(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));

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

      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      generator.getOrientation(orientationToPack);
      generator.setContinuouslyUpdateFinalOrientation(true);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector3D angularVelocityToPack = new FrameVector3D(referenceFrame, 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(referenceFrame, 10.0, 10.0, 10.0);

      assertTrue(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));

      assertTrue(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertFalse(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));

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

//   private YoVariableRegistry createRegistry()
//   {
//      YoVariableRegistry registry = new YoVariableRegistry("registry" + globalCounter);
//      globalCounter++;
//      return registry;
//   }
}
