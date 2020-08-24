package us.ihmc.simpleWholeBodyWalking.simpleSphere;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.commonWalkingControlModules.orientationControl.VariationalLQRController;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;

public class SimpleLQRSphereController implements SimpleSphereControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereLQRController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SimpleSphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRMomentumController lqrMomentumController;
   private final VariationalLQRController variationalLQRController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D lqrTorque = new YoFrameVector3D("lqrTorque", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   
   private final YoBoolean leftInContact = new YoBoolean("LeftInContact", registry);
   private final YoBoolean rightInContact = new YoBoolean("RightInContact", registry);
   
   private final List<RobotSide> currentFeetInContact = new ArrayList<>();
   
   private final SimpleSphereVisualizer vizSphere;

   private final List<Footstep> footstepList = new ArrayList<>();
   private final List<FootstepTiming> footstepTimingList = new ArrayList<>();
   private boolean isDoubleSupport;

   public SimpleLQRSphereController(SimpleSphereRobot sphereRobot, SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scsRobot = sphereRobot.getScsRobot();
      this.sphereRobot = sphereRobot;
      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);

      dcmPlan = comTrajectoryProvider;

      sphereRobot.getScsRobot().setController(this);

      lqrMomentumController = new LQRMomentumController(sphereRobot.getOmega0Provider(), registry);
      variationalLQRController = new VariationalLQRController();
      variationalLQRController.setInertia(sphereRobot.getInertiaTensor());
      
      vizSphere = new SimpleSphereVisualizer(dcmPlan, yoGraphicsListRegistry, sphereRobot, registry);
      
      dcmPlan.initialize();
      
      //start in transfer
      isDoubleSupport = false;
      dcmPlan.initializeForStanding(0);
   }

   private final DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
   private Quaternion currentRotation = new Quaternion();
   private FrameVector3D currentAngularVelocity = new FrameVector3D();
   private double angleGain = 1;
   private double yawDesired;
   private double pitchDesired; 
   private double rollDesired;
   private double yawCurrent;
   private double pitchCurrent; 
   private double rollCurrent;
   private Quaternion desiredRotation = new Quaternion(0,0,0,1);

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.updateFrames();

      double currentTime = sphereRobot.getScsRobot().getYoTime().getDoubleValue();
      updateFeetState(currentTime);
      updateFeetYoVar();
      dcmPlan.setInitialCenterOfMassState(sphereRobot.getCenterOfMass(), sphereRobot.getCenterOfMassVelocity());
      double timeInPhase = dcmPlan.computeSetpoints(currentTime, footstepList, footstepTimingList);

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      lqrMomentumController.setVRPTrajectory(dcmPlan.getVRPTrajectories());
      sphereRobot.getCenterOfMass().get(currentState);
      sphereRobot.getCenterOfMassVelocity().get(3, currentState);
      lqrMomentumController.computeControlInput(currentState, timeInPhase);

      lqrForce.set(lqrMomentumController.getU());
      lqrForce.addZ(sphereRobot.getGravityZ());
      lqrForce.scale(sphereRobot.getTotalMass());

      externalForcePoint.setForce(lqrForce);
      
      currentRotation.set(sphereRobot.getCurrentRotation());
      yawCurrent = currentRotation.getYaw();
      pitchCurrent = currentRotation.getPitch();
      rollCurrent = currentRotation.getRoll();
      currentAngularVelocity.setIncludingFrame(sphereRobot.getAngularVelocity());
      currentAngularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      if (currentTime < 6)
      {
         yawDesired = 0;
         pitchDesired = 0;
         rollDesired = 0.6;
      }
      else
      {
         yawDesired = 0;
         pitchDesired = 0;
         rollDesired = 0;
      }
      desiredRotation.setYawPitchRoll(yawDesired, pitchDesired, rollDesired);
      variationalLQRController.setDesired(desiredRotation, new Vector3D(), new Vector3D());
      variationalLQRController.compute(currentRotation, currentAngularVelocity);
      
      Vector3D torqueVector = new Vector3D();
      variationalLQRController.getDesiredTorque(torqueVector);
      lqrTorque.set(torqueVector);
      
      //externalForcePoint.setMoment(torqueVector);
      
      
      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
      
      vizSphere.updateVizPoints(currentTime, lqrForce);
      vizSphere.updateVizFeet(currentTime, currentFeetInContact, footstepList, footstepTimingList);
   }

  private void updateFeetYoVar()
   {
     leftInContact.set(false);
     rightInContact.set(false);
      for(int i=0; i<currentFeetInContact.size(); i++)
      {
         if (currentFeetInContact.get(i) == RobotSide.LEFT)
            leftInContact.set(true);
         if (currentFeetInContact.get(i) == RobotSide.RIGHT)
            rightInContact.set(true);
      }
      
   }

  private void updateFeetState(double currentTime)
  {
     currentFeetInContact.clear();
     
     if(footstepList.size() == 0)
     {//Simulation has finished all planned steps
        for (RobotSide robotSide : RobotSide.values)
           currentFeetInContact.add(robotSide);
        return;
     }
     
     //Simulation is in initial transfer
     if(currentTime < footstepTimingList.get(0).getExecutionStartTime())
     {
        if (!isDoubleSupport)
        {
           dcmPlan.initializeForTransfer(currentTime);
           isDoubleSupport = true;
        }
        
        for (RobotSide robotSide : RobotSide.values)
           currentFeetInContact.add(robotSide);
        return;
     }
     
     for (int i = 0; i < footstepTimingList.size(); i++)
     {
        double swingStartTime = footstepTimingList.get(i).getExecutionStartTime() + footstepTimingList.get(i).getSwingStartTime();
        double swingEndTime = swingStartTime + footstepTimingList.get(i).getSwingTime();
        double footstepEndTime = swingEndTime + footstepTimingList.get(i).getTransferTime();
        
        if (currentTime >= swingStartTime && currentTime < swingEndTime)
        {
           //Robot is in swing
           if (isDoubleSupport)
           {
              dcmPlan.initializeForSingleSupport(currentTime);
              dcmPlan.setSupportLeg(footstepList.get(i).getRobotSide().getOppositeSide());
              isDoubleSupport = false;
           }
           currentFeetInContact.add(footstepList.get(i).getRobotSide().getOppositeSide());
           sphereRobot.updateSoleFrame(footstepList.get(i).getRobotSide(), footstepList.get(i).getFootstepPose().getPosition());               
           return;
        }
        else if (currentTime >= swingEndTime && currentTime < footstepEndTime)
        {
           //Robot is in transfer after swing
           if (!isDoubleSupport)
           {
              dcmPlan.initializeForTransfer(currentTime);
              isDoubleSupport = true;
              footstepList.remove(i);
              footstepTimingList.remove(i);
              dcmPlan.setTransferToSide(footstepList.get(i).getRobotSide());
           }
           currentFeetInContact.add(footstepList.get(i).getRobotSide().getOppositeSide());
           sphereRobot.updateSoleFrame(footstepList.get(i).getRobotSide(), footstepList.get(i).getFootstepPose().getPosition());               
           return;
        }
     }
  }

  @Override
  public void setFootstepPlan(List<Footstep> footstepList, List<FootstepTiming> footstepTimingList)
  {
     this.footstepList.clear();
     this.footstepTimingList.clear();
     this.footstepList.addAll(footstepList);
     this.footstepTimingList.addAll(footstepTimingList);
     dcmPlan.setFinalTransferDuration(footstepTimingList.get(0).getExecutionStartTime());
  }
  
  @Override
  public void initialize()
  {
  }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

}
