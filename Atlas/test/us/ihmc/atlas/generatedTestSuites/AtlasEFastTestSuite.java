package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.ObstacleCourseTests.AtlasObstacleCourseTrialsWalkingTaskTest.class,
   us.ihmc.atlas.ObstacleCourseTests.AtlasWallWorldTest.class,
   us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.roughTerrainWalking.AtlasFootExplorationTest.class,
   us.ihmc.atlas.stateEstimation.AtlasPelvisPoseHistoryCorrectorTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class,
   us.ihmc.atlas.WholeBodyIkSolverTest.class
})

public class AtlasEFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasEFastTestSuite.class);
   }
}

