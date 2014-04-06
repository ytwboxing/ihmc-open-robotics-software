package us.ihmc.valkyrie.paramaters;

import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public class ValkyrieStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final boolean useKinematicsBasedStateEstimator = true;
   private final boolean assumePerfectIMU = true;

   private final double kinematicsPelvisLinearVelocityFilterFreqInHertz;
   private final double kinematicsPelvisPositionFilterFreqInHertz;
   
   private final double jointVelocitySlopTimeForBacklashCompensation;

   private final double jointPositionFilterFrequencyHz;
   private final double jointVelocityFilterFrequencyHz;
   private final double orientationFilterFrequencyHz;
   private final double angularVelocityFilterFrequencyHz;
   private final double linearAccelerationFilterFrequencyHz;

   // State Estimator Filter Parameters
   private final double pointVelocityXYMeasurementStandardDeviation;
   private final double pointVelocityZMeasurementStandardDeviation;

   private final double pointPositionXYMeasurementStandardDeviation;
   private final double pointPositionZMeasurementStandardDeviation;

   private final boolean useTwoPolesForIMUFiltering;
   private final boolean doFiniteDifferenceForJointVelocities;

   private final SensorFilterParameters sensorFilterParameters;

   private final PointMeasurementNoiseParameters pointMeasurementNoiseParameters;

   //      DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
   private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters
         .createNoiseParametersForEstimatorJerryTuningSeptember2013();

   public ValkyrieStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      final double defaultFilterBreakFrequency;

      if (!runningOnRealRobot)
      {
         defaultFilterBreakFrequency = Double.POSITIVE_INFINITY;
      }
      else
      {
         defaultFilterBreakFrequency = Double.POSITIVE_INFINITY; //16.0;
      }

      jointPositionFilterFrequencyHz = defaultFilterBreakFrequency;
      jointVelocityFilterFrequencyHz = defaultFilterBreakFrequency;
      orientationFilterFrequencyHz = defaultFilterBreakFrequency;
      angularVelocityFilterFrequencyHz = defaultFilterBreakFrequency;
      linearAccelerationFilterFrequencyHz = defaultFilterBreakFrequency;

      jointVelocitySlopTimeForBacklashCompensation = 0.03;

      pointVelocityXYMeasurementStandardDeviation = 2.0;
      pointVelocityZMeasurementStandardDeviation = 2.0;

      pointPositionXYMeasurementStandardDeviation = 0.1;
      pointPositionZMeasurementStandardDeviation = 0.1;

      useTwoPolesForIMUFiltering = runningOnRealRobot;
      doFiniteDifferenceForJointVelocities = false;

      sensorFilterParameters = new SensorFilterParameters(jointPositionFilterFrequencyHz, jointVelocityFilterFrequencyHz, orientationFilterFrequencyHz,
            angularVelocityFilterFrequencyHz, linearAccelerationFilterFrequencyHz, jointVelocitySlopTimeForBacklashCompensation, estimatorDT, useTwoPolesForIMUFiltering, doFiniteDifferenceForJointVelocities);

      pointMeasurementNoiseParameters = new PointMeasurementNoiseParameters(pointVelocityXYMeasurementStandardDeviation,
            pointVelocityZMeasurementStandardDeviation, pointPositionXYMeasurementStandardDeviation, pointPositionZMeasurementStandardDeviation);
      
      kinematicsPelvisPositionFilterFreqInHertz = Double.POSITIVE_INFINITY;
      kinematicsPelvisLinearVelocityFilterFreqInHertz = 16.0;
   }

   @Override
   public SensorFilterParameters getSensorFilterParameters()
   {
      return sensorFilterParameters;
   }

   @Override
   public boolean getAssumePerfectIMU()
   {
      return assumePerfectIMU;
   }

   @Override
   public boolean useKinematicsBasedStateEstimator()
   {
      return useKinematicsBasedStateEstimator;
   }

   @Override
   public PointMeasurementNoiseParameters getPointMeasurementNoiseParameters()
   {
      return pointMeasurementNoiseParameters;
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return sensorNoiseParameters;
   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return kinematicsPelvisPositionFilterFreqInHertz;
   }
   
   @Override
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return kinematicsPelvisLinearVelocityFilterFreqInHertz;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return 4.0;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return false;
   }

   @Override
   public boolean useHackishAccelerationIntegration()
   {
      return false;
   }

   @Override
   public boolean estimateGravity()
   {
      return false;
   }

   @Override
   public double getGravityFilterFreqInHertz()
   {
      return 5.3052e-4;
   }

   @Override
   public double getPelvisPositionFusingFrequency()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getPelvisLinearVelocityFusingFrequency()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getDelayTimeForTrustingFoot()
   {
      return 0.02;
   }

   @Override
   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      return 0.3;
   }

   @Override
   public boolean estimateIMUDrift()
   {
      return false;
   }

   @Override
   public boolean compensateIMUDrift()
   {
      return false;
   }

   @Override
   public double getIMUDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   @Override
   public double getFootVelocityUsedForImuDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   @Override
   public double getFootVelocityThresholdToEnableIMUDriftCompensation()
   {
      return 0.03;
   }

   @Override
   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return false;
   }
}
