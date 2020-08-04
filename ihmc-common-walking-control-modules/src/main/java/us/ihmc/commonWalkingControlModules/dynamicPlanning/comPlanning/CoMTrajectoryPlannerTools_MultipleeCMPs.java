package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.List;

public class CoMTrajectoryPlannerTools_MultipleeCMPs
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double minDuration = 1.0e-5;
   public static final double sufficientlyLarge = 1.0e10;
   public static final double sufficientlyLongTime = 1.0e2;
   public static final double sufficientlyLargeThird = Math.pow(1.0e10, 1.0 / 3.0);
   public static int matrixIndex;
   
   /*
    * constraintMatrixToPack has 18 coefficients to solve for as follows:
    *    c0    - first coefficient for CoM position
    *    c1    - second coefficient for CoM position
    *    c2    - third coefficient for CoM position, VRP
    *    c3    - fourth coefficient for CoM position, VRP
    *    c4    - fifth coefficient for CoM position, VRP
    *    c5    - sixth coefficient for CoM position, VRP
    *    c0,l  - first coefficient for eCMP_left
    *    c1,l  - second coefficient for eCMP_left
    *    c2,l  - third coefficient for eCMP_left
    *    c3,l  - fourth coefficient for eCMP_left
    *    c0,r  - first coefficient for eCMP_right
    *    c1,r  - second coefficient for eCMP_right
    *    c2,r  - third coefficient for eCMP_right
    *    c3,r  - fourth coefficient for eCMP_right
    *    a0    - first coefficient for Computed CoM
    *    a1    - second coefficient for Computed CoM
    *    a2    - third coefficient for Computed CoM
    *    a3    - fourth coefficient for Computed CoM
    */
   
   public static void setMatrixIndex(int curMatrixIndex) {
      matrixIndex = curMatrixIndex;
   }
   
   public static void computeVRPWaypoints(double nominalCoMHeight, double gravityZ, double omega, FrameVector3DReadOnly currentCoMVelocity,
                                          List<? extends ContactStateProvider> contactSequence, RecyclingArrayList<FramePoint3D> startVRPPositionsToPack,
                                          RecyclingArrayList<FramePoint3D> endVRPPositionsToPack)
   {
      computeVRPWaypoints(nominalCoMHeight, gravityZ, omega, currentCoMVelocity, contactSequence, startVRPPositionsToPack, endVRPPositionsToPack, true);
   }

   public static void computeVRPWaypoints(double nominalCoMHeight, double gravityZ, double omega, FrameVector3DReadOnly currentCoMVelocity,
                                          List<? extends ContactStateProvider> contactSequence, RecyclingArrayList<FramePoint3D> startVRPPositionsToPack,
                                          RecyclingArrayList<FramePoint3D> endVRPPositionsToPack, boolean adjustWaypointHeightForHeightChange)
   {
      startVRPPositionsToPack.clear();
      endVRPPositionsToPack.clear();

      double initialHeightVelocity = currentCoMVelocity.getZ();
      double finalHeightVelocity;

      for (int i = 0; i < contactSequence.size() - 1; i++)
      {
         ContactStateProvider contactStateProvider = contactSequence.get(i);
         boolean finalContact = i == contactSequence.size() - 1;
         ContactStateProvider nextContactStateProvider = null;
         if (!finalContact)
            nextContactStateProvider = contactSequence.get(i + 1);

         FramePoint3D start = startVRPPositionsToPack.add();
         FramePoint3D end = endVRPPositionsToPack.add();

         start.set(contactStateProvider.getCopStartPosition());
         start.addZ(nominalCoMHeight);
         end.set(contactStateProvider.getCopEndPosition());
         end.addZ(nominalCoMHeight);

         if (adjustWaypointHeightForHeightChange)
         {
            double duration = contactStateProvider.getTimeInterval().getDuration();
            duration = Math.signum(duration) * Math.max(Math.abs(duration), minDuration);
            if (!contactStateProvider.getContactState().isLoadBearing())
            {
               finalHeightVelocity = initialHeightVelocity - gravityZ * duration;
            }
            else
            {
               if (!finalContact && !nextContactStateProvider.getContactState().isLoadBearing())
               { // next is a jump, current one is load bearing
                  ContactStateProvider nextNextContactStateProvider = contactSequence.get(i + 2);
                  double heightBeforeJump = contactStateProvider.getCopEndPosition().getZ();
                  double finalHeightAfterJump = nextNextContactStateProvider.getCopStartPosition().getZ();

                  double heightChangeWhenJumping = finalHeightAfterJump - heightBeforeJump;
                  double durationOfJump = nextContactStateProvider.getTimeInterval().getDuration();

                  /* delta z = v0 T - 0.5 g T^2
                   * v0 =  delta z / T + 0.5 g T**/
                  finalHeightVelocity = heightChangeWhenJumping / durationOfJump + 0.5 * gravityZ * durationOfJump;
               }
               else
               { // next is is load bearing, current is load bearing.
                  finalHeightVelocity = 0.0;
               }
            }

            // offset the height VRP waypoint based on the desired velocity change
            double heightVelocityChange = finalHeightVelocity - initialHeightVelocity;
            double offset = heightVelocityChange / (MathTools.square(omega) * duration);
            start.subZ(offset);
            end.subZ(offset);

            initialHeightVelocity = finalHeightVelocity;
         }
      }

      ContactStateProvider contactStateProvider = contactSequence.get(contactSequence.size() - 1);

      FramePoint3D start = startVRPPositionsToPack.add();
      FramePoint3D end = endVRPPositionsToPack.add();

      start.set(contactStateProvider.getCopStartPosition());
      start.addZ(nominalCoMHeight);
      end.set(contactStateProvider.getCopEndPosition());
      end.addZ(nominalCoMHeight);
   }

   /**
    * <p> Sets the continuity constraint on the initial CoM position. This DOES result in a initial discontinuity on the desired DCM location,
    * coming from a discontinuity on the desired CoM Velocity. </p>
    * <p> This constraint should be used for the initial position of the center of mass to properly initialize the trajectory. </p>
    * <p> Recall that the equation for the center of mass is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p>
    *    This constraint defines
    * </p>
    * <p>
    *    x<sub>0</sub>(0) = x<sub>d</sub>,
    * </p>
    * <p>
    *    substituting in the coefficients into the constraint matrix.
    * </p>
    * @param centerOfMassLocationForConstraint x<sub>d</sub> in the above equations
    */
   public static void addCoMPositionConstraint(FramePoint3DReadOnly centerOfMassLocationForConstraint, double omega, double time, int sequenceId, int constraintNumber,
                                               DMatrixRMaj constraintMatrixToPack, DMatrixRMaj xObjectiveMatrixToPack,
                                               DMatrixRMaj yObjectiveMatrixToPack, DMatrixRMaj zObjectiveMatrixToPack)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);

      int startIndex = matrixIndex * sequenceId;
      constraintMatrixToPack.set(constraintNumber, startIndex, getCoMPositionFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, getCoMPositionSecondCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, getCoMPositionThirdCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, getCoMPositionFourthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, getCoMPositionFifthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, getCoMPositionSixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.add(constraintNumber, 0, centerOfMassLocationForConstraint.getX());
      yObjectiveMatrixToPack.add(constraintNumber, 0, centerOfMassLocationForConstraint.getY());
      zObjectiveMatrixToPack.add(constraintNumber, 0, centerOfMassLocationForConstraint.getZ());
   }

   /**
    * <p> Sets a constraint on the desired DCM position. This constraint is useful for constraining the terminal location of the DCM trajectory. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    *    d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    *    &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    *     2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>
    * </p>
    * <p>
    *    This constraint is then combining these two, saying
    * </p>
    * <p> x<sub>i</sub>(t<sub>i</sub>) + 1 / &omega; d/dt x<sub>i</sub>(t<sub>i</sub>) = &xi;<sub>d</sub>,</p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId i in the above equations
    * @param time t<sub>i</sub> in the above equations
    * @param desiredDCMPosition desired DCM location. &xi;<sub>d</sub> in the above equations.
    */
   public static void addDCMPositionConstraint(int sequenceId, int constraintNumber, double time, double omega, FramePoint3DReadOnly desiredDCMPosition,
                                               DMatrixRMaj constraintMatrixToPack, DMatrixRMaj xObjectiveMatrixToPack,
                                               DMatrixRMaj yObjectiveMatrixToPack, DMatrixRMaj zObjectiveMatrixToPack)
   {
      desiredDCMPosition.checkReferenceFrameMatch(worldFrame);

      int startIndex = matrixIndex * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      // add constraints on terminal DCM position
      constraintMatrixToPack.set(constraintNumber, startIndex, getDCMPositionFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, getDCMPositionSecondCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, getDCMPositionThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, getDCMPositionFourthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, getDCMPositionFifthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, getDCMPositionSixthCoefficientTimeFunction());
      
      xObjectiveMatrixToPack.add(constraintNumber, 0, desiredDCMPosition.getX());
      yObjectiveMatrixToPack.add(constraintNumber, 0, desiredDCMPosition.getY());
      zObjectiveMatrixToPack.add(constraintNumber, 0, desiredDCMPosition.getZ());
   }

   /**
    * <p> Adds a constraint for the desired VRP position.</p>
    * <p> Recall that the VRP is defined as </p>
    * <p> v<sub>i</sub>(t<sub>i</sub>) =  c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * (c<sub>4,i</sub> - 6/&omega;<sup>2</sup> c<sub>2,i</sub>) t<sub>i</sub> - 2/&omega; c<sub>3,i</sub> + c<sub>5,i</sub></p>.
    * <p> This constraint then says </p>
    * <p> v<sub>i</sub>(t<sub>i</sub>) = J v<sub>d</sub> </p>
    * <p> where J is a Jacobian that maps from a vector of desired VRP waypoints to the constraint form, and </p>
    * <p> v<sub>d,j</sub> = v<sub>r</sub> </p>
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointPositionIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPPosition reference VRP position, v<sub>r</sub> in the above equations.
    */
   public static void addVRPPositionConstraint(int sequenceId, int constraintNumber, int vrpWaypointPositionIndex, double time, double omega,
                                               FramePoint3DReadOnly desiredVRPPosition, DMatrixRMaj constraintMatrixToPack,
                                               DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack,
                                               DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj vrpWaypointJacobianToPack)
   {
      int startIndex = matrixIndex * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      desiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      constraintMatrixToPack.set(constraintNumber, startIndex + 0, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPPositionFirstCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPPositionSecondCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPPositionThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPPositionFourthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPPositionFifthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPPositionSixthCoefficientTimeFunction());
      
      vrpWaypointJacobianToPack.set(constraintNumber, vrpWaypointPositionIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getX());
      yObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getY());
      zObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getZ());
   }

   /**
    * <p> Adds a constraint for the desired VRP velocity.</p>
    * <p> Recall that the VRP velocity is defined as </p>
    * <p> d/dt v<sub>i</sub>(t<sub>i</sub>) =  3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> + 2 c<sub>3,i</sub> t<sub>i</sub> +
    * (c<sub>4,i</sub> - 6/&omega;<sup>2</sup> c<sub>2,i</sub>).
    * <p> This constraint then says </p>
    * <p> d/dt v<sub>i</sub>(t<sub>i</sub>) = J v<sub>d</sub> </p>
    * <p> where J is a Jacobian that maps from a vector of desired VRP waypoints to the constraint form, and </p>
    * <p> v<sub>d,j</sub> = d/dt v<sub>r</sub> </p>
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointVelocityIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPVelocity reference VRP veloctiy, d/dt v<sub>r</sub> in the above equations.
    */
   public static void addVRPVelocityConstraint(int sequenceId, int constraintNumber, int vrpWaypointVelocityIndex, double omega, double time,
                                               FrameVector3DReadOnly desiredVRPVelocity, DMatrixRMaj constraintMatrixToPack,
                                               DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack,
                                               DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj vrpWaypointJacobianToPack)
   {
      int startIndex = matrixIndex * sequenceId;

      desiredVRPVelocity.checkReferenceFrameMatch(worldFrame);

      constraintMatrixToPack.set(constraintNumber, startIndex + 0, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPVelocityFirstCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPVelocitySecondCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPVelocityThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPVelocityFourthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPVelocityFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, CoMTrajectoryPlannerTools_MultipleeCMPs.getVRPVelocitySixthCoefficientTimeFunction());
      
      vrpWaypointJacobianToPack.set(constraintNumber, vrpWaypointVelocityIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getX());
      yObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getY());
      zObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getZ());
   }

   /**
    * <p> Set a continuity constraint on the CoM position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMPositionContinuityConstraint(int previousSequence, int nextSequence, int constraintNumber, double omega, double previousDuration,
                                                         DMatrixRMaj constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = matrixIndex * previousSequence;
      int nextStartIndex = matrixIndex * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintNumber, previousStartIndex, getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 1, getCoMPositionSecondCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 2, getCoMPositionThirdCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 3, getCoMPositionFourthCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 4, getCoMPositionFifthCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 5, getCoMPositionSixthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, nextStartIndex, -getCoMPositionFirstCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 1, -getCoMPositionSecondCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 2, -getCoMPositionThirdCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 3, -getCoMPositionFourthCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 4, -getCoMPositionFifthCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 5, -getCoMPositionSixthCoefficientTimeFunction());
   }

   /**
    * <p> Set a continuity constraint on the CoM velocity at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    *    &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    *     2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> d / dt x<sub>i-1</sub>(T<sub>i-1</sub>) = d / dt x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMVelocityContinuityConstraint(int previousSequence, int nextSequence, int constraintNumber, double omega, double previousDuration,
                                                         DMatrixRMaj constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = matrixIndex * previousSequence;
      int nextStartIndex = matrixIndex * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintNumber, previousStartIndex, getCoMVelocityFirstCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 1, getCoMVelocitySecondCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 2, getCoMVelocityThirdCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 3, getCoMVelocityFourthCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 4, getCoMVelocityFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 5, getCoMVelocitySixthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, nextStartIndex, -getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 1, -getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 2, -getCoMVelocityThirdCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 3, -getCoMVelocityFourthCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 4, -getCoMVelocityFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 5, -getCoMVelocitySixthCoefficientTimeFunction());
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have an acceleration equal to gravity at time t.</p>
    * <p> Recall that the CoM acceleration is defined as </p>
    * d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>2</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> +
    * &omega;<sup>2</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub> t<sub>i</sub> + 2 c<sub>3,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = -g, </p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainCoMAccelerationToGravity(int sequenceId, int constraintNumber, double omega, double time, double gravityZ,
                                                        DMatrixRMaj constraintMatrixToPack, DMatrixRMaj zObjectiveMatrixToPack)
   {
      int startIndex = matrixIndex * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintNumber, startIndex, getCoMAccelerationFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, getCoMAccelerationSecondCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, getCoMAccelerationThirdCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, getCoMAccelerationFourthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, getCoMAccelerationFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, getCoMAccelerationSixthCoefficientTimeFunction());

      zObjectiveMatrixToPack.set(constraintNumber, 0, -Math.abs(gravityZ));
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have a jerk equal to 0.0 at time t.</p>
    * <p> Recall that the CoM jerk is defined as </p>
    * d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>3</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega;<sup>3</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = 0.0, </p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainCoMJerkToZero(double time, double omega, int sequenceId, int constraintNumber, DMatrixRMaj constraintMatrixToPack)
   {
      time = Math.min(time, sufficientlyLongTime);

      int startIndex = matrixIndex * sequenceId;
      constraintMatrixToPack.set(constraintNumber, startIndex, getCoMJerkFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, getCoMJerkSecondCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, getCoMJerkThirdCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, getCoMJerkFourthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, getCoMJerkFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, getCoMJerkSixthCoefficientTimeFunction());
   }
   
   /**
    * <p> Adds 4 constraints for the left/right eCMPs during a single support state for a left-to-right footstep.</p>
    * <p> For a right-to-left footstep, the following conditions are set in this method. </p>
    * <p> C<sub>1,l</sub> = x(0), c<sub>1,r</sub> = r<sub>ecmp,T</sub>, c<sub>0,l</sub>T + c<sub>1,l</sub> = x(T), and 
    *     c<sub>0,r</sub>T + c<sub>1,r</sub> = r<sub>ecmp,H</sub>
    * <p> More details can be found in setECMPConstraints() method in CoMTrajectoryPlanner_MultipleeCMPs.java file. </p>
    * <p> Because the left-to-right and right-to-left footsteps are just flipping constants depending on the conditions. </p>
    * <p> The getECMPStart_0,1_XCoefficient and getECPMEnd_0,1_XCoefficient methods take care of the flipping constants. The start
    *     and end refer to which eCMP will start at the VRP position and which one will end at the VRP position. </p>
    * <p> The negative is flipped depending on the condition in getECMPLeft_0,1_XCoefficient and get ECMPRight_0,1_XCofficient. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainECMPsForDoubleLeftToRightSupportStep(double time, double omega, int sequenceId, int constraintNumber, 
                                                       FramePoint3DReadOnly desiredVRPStartPosition,  FramePoint3DReadOnly desiredVRPEndPosition, 
                                                       DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                                       DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj constraintMatrixToPack) {
      time = Math.min(time, sufficientlyLongTime);

      int startIndex = matrixIndex * sequenceId;
      
      /*
       * eCMP_left   = C_l0*t + C_l1
       * eCMP_right  = C_r0*t + C_r1
       */
      
      // constraint left eCMP first constant (C_10)
      constraintMatrixToPack.set(constraintNumber, startIndex,       getECMPStart_0_FirstCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 1,   getECMPStart_0_SecondCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 4,   getECMPStart_0_ThirdCoefficient(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5,   getECMPStart_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 8,   -getECMPLeft_0_FirstCoefficient(time, 1.0));
      constraintMatrixToPack.set(constraintNumber, startIndex + 9,   -getECMPLeft_0_ThirdCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber, startIndex + 10,  0.0); // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 11,  0.0); // c1,r 
      constraintMatrixToPack.set(constraintNumber, startIndex + 12,  0.0); // c2,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 13,  0.0); // c3,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  0.0); // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  0.0); // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  0.0); // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  0.0); // a3

      // constrain right eCMP first constant (C_r0)
      constraintMatrixToPack.set(constraintNumber + 1, startIndex,       getECMPEnd_0_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 1,   getECMPEnd_0_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 4,   getECMPEnd_0_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 5,   getECMPEnd_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 8,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 9,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 12,  getECMPRight_0_SecondCoefficient(time, 1.0));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 13,  getECMPRight_0_FourthCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 17,  0.0);
      
      // constrain left eCMP second constant (C_l1)
      constraintMatrixToPack.set(constraintNumber + 2, startIndex,       getECMPStart_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 1,   getECMPStart_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 4,   getECMPStart_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 5,   getECMPStart_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 8,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 9,   getECMPLeft_1_ThirdCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 17,  0.0);
      
      // constrain right eCMP second constant (C_r1)
      constraintMatrixToPack.set(constraintNumber + 3, startIndex,       getECMPEnd_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 1,   getECMPEnd_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 4,   getECMPEnd_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 5,   getECMPEnd_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 8,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 9,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 13,  getECMPRight_1_FourthCoefficient(-1.0));
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 17,  0.0);
         
      /*
       *  Set objective functions.
       */
      
      // Equals zero
      
      xObjectiveMatrixToPack.set(constraintNumber + 1, 0, desiredVRPEndPosition.getX());
      yObjectiveMatrixToPack.set(constraintNumber + 1, 0, desiredVRPEndPosition.getY());
      zObjectiveMatrixToPack.set(constraintNumber + 1, 0, desiredVRPEndPosition.getZ());
      
      xObjectiveMatrixToPack.set(constraintNumber + 2, 0, desiredVRPStartPosition.getX());
      yObjectiveMatrixToPack.set(constraintNumber + 2, 0, desiredVRPStartPosition.getY());
      zObjectiveMatrixToPack.set(constraintNumber + 2, 0, desiredVRPStartPosition.getZ());
      
      // Equals zero
      }
   
   /**
    * <p> Adds 4 constraints for the left/right eCMPs during a single support state for a right-to-left footstep.</p>
    * <p> For a right-to-left footstep, the following conditions are set in this method. </p>
    * <p> C<sub>1,l</sub> = x(0), c<sub>1,r</sub> = r<sub>ecmp,T</sub>, c<sub>0,l</sub>T + c<sub>1,l</sub> = x(T), and 
    *     c<sub>0,r</sub>T + c<sub>1,r</sub> = r<sub>ecmp,H</sub>
    * <p> More details can be found in setECMPConstraints() method in CoMTrajectoryPlanner_MultipleeCMPs.java file. </p>
    * <p> Because the left-to-right and right-to-left footsteps are just flipping constants depending on the conditions. </p>
    * <p> The getECMPStart_0,1_XCoefficient and getECPMEnd_0,1_XCoefficient methods take care of the flipping constants. The start
    *     and end refer to which eCMP will start at the VRP position and which one will end at the VRP position. </p>
    * <p> The negative is flipped depending on the condition in getECMPLeft_0,1_XCoefficient and get ECMPRight_0,1_XCofficient. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainECMPsForDoubleRightToLeftSupportStep(double time, double omega, int sequenceId, int constraintNumber, 
                                                        FramePoint3DReadOnly desiredVRPStartPosition,  FramePoint3DReadOnly desiredVRPEndPosition, 
                                                        DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                                        DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj constraintMatrixToPack) {
      time = Math.min(time, sufficientlyLongTime);

      int startIndex = matrixIndex * sequenceId;
      
      /*
       * eCMP_left   = c2,l*t + c3,l 
       * c0,l = c1,l = 0
       * eCMP_right  = c2,r*t + c3,r
       * c0,r = c1,r = 0
       */
      
      // constrain left eCMP first constant (C_l0)
      constraintMatrixToPack.set(constraintNumber, startIndex,       getECMPEnd_0_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 1,   getECMPEnd_0_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 4,   getECMPEnd_0_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 5,   getECMPEnd_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 8,   getECMPLeft_0_FirstCoefficient(time, 1.0));
      constraintMatrixToPack.set(constraintNumber, startIndex + 9,   getECMPLeft_0_ThirdCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber, startIndex + 10,  0.0); // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 11,  0.0); // c1,r 
      constraintMatrixToPack.set(constraintNumber, startIndex + 12,  0.0); // c2,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 13,  0.0); // c3,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  0.0); // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  0.0); // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  0.0); // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  0.0); // a3
      
      // constraint right eCMP first constant (C_r0)
      constraintMatrixToPack.set(constraintNumber + 1, startIndex,       getECMPStart_0_FirstCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 1,   getECMPStart_0_SecondCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 4,   getECMPStart_0_ThirdCoefficient(time));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 5,   getECMPStart_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 8,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 9,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 12,  -getECMPRight_0_SecondCoefficient(time, 1.0));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 13,  -getECMPRight_0_FourthCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 17,  0.0);
      
      // constrain left eCMP second constant (C_l1)
      constraintMatrixToPack.set(constraintNumber + 2, startIndex,       getECMPEnd_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 1,   getECMPEnd_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 4,   getECMPEnd_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 5,   getECMPEnd_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 8,   -getECMPLeft_1_ThirdCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 9,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 17,  0.0);
      
      // constrain right eCMP second constant (C_r1)
      constraintMatrixToPack.set(constraintNumber + 3, startIndex,       getECMPStart_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 1,   getECMPStart_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 4,   getECMPStart_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 5,   getECMPStart_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 7,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 8,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 9,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 13,  getECMPRight_1_FourthCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 17,  0.0);
      
      /*
       * Set Objective Functions.
       */
      
      xObjectiveMatrixToPack.set(constraintNumber, 0, desiredVRPEndPosition.getX());
      yObjectiveMatrixToPack.set(constraintNumber, 0, desiredVRPEndPosition.getY());
      zObjectiveMatrixToPack.set(constraintNumber, 0, desiredVRPEndPosition.getZ());
      
      // equals zero
      
      // equals zero
      
      xObjectiveMatrixToPack.set(constraintNumber + 3, 0, desiredVRPStartPosition.getX());
      yObjectiveMatrixToPack.set(constraintNumber + 3, 0, desiredVRPStartPosition.getY());
      zObjectiveMatrixToPack.set(constraintNumber + 3, 0, desiredVRPStartPosition.getZ());
   }
   
   /**
    * <p> Adds 4 constraints for the left/right eCMPs during a double support state before a left footstep.</p>
    * <p> Double support states take place between two single support states. The eCMPs will be in different conditions 
    *     depending on the which footstep is prior and which one comes next. This method connects the end and start conditions
    *     to keep the eCMPs continuous. </p>
    * <p> For a right-to-left footstep, the right eCMP, eCMP<sub>r</sub>, begins at the VRP right position (VRP<sub>start</sub>) and
    *     ends at the CoM end position, x(T). The left eCMP (eCMP<sub>l</sub>) begins at the CoM, x(0), and ends at the VRP left position,
    *     VRP<sub>end</sub>. </p>
    * <p> At that footstep, this method takes care of the double support state to start a left footstep. The left eCMP would stay at the
    *     VRP during this duration and the right eCMP would follow the CoM to maintain continuity. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainECMPsForSingleLeftSupportStep(double time, double omega, int sequenceId, int constraintNumber, 
                                                                    FramePoint3DReadOnly desiredVRPStartPosition,  
                                                                    DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                                                    DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj constraintMatrixToPack) {
      time = Math.min(time, sufficientlyLongTime);

      int startIndex = matrixIndex * sequenceId;
      
      /*
       * eCMP_left   = c0,l*exp(omega*t) + c1,l*exp(-omega*t) + c2,l*t + c3,l
       * eCMP_right  = c0,r*exp(omega*t) + c1,r*exp(-omega*t) + c2,r*t + c3,r
       * 
       * This function actually uses c0,l , c1,l , c0,r and c1,r
       */
      
      // constrain left eCMP first constant (C_l0)
      constraintMatrixToPack.set(constraintNumber, startIndex,          0.0); // c0
      constraintMatrixToPack.set(constraintNumber, startIndex + 1,      0.0); // c1
      constraintMatrixToPack.set(constraintNumber, startIndex + 2,      0.0); // c2 x only for flight
      constraintMatrixToPack.set(constraintNumber, startIndex + 3,      0.0); // c3 x only for flight
      constraintMatrixToPack.set(constraintNumber, startIndex + 4,      0.0); // c4
      constraintMatrixToPack.set(constraintNumber, startIndex + 5,      0.0); // c5
      constraintMatrixToPack.set(constraintNumber, startIndex + 6,      1.0); // c0,l follow com during single support
      constraintMatrixToPack.set(constraintNumber, startIndex + 7,      0.0); // c1,l follow com during single support
      constraintMatrixToPack.set(constraintNumber, startIndex + 8,      0.0); // c2,l
      constraintMatrixToPack.set(constraintNumber, startIndex + 9,      0.0); // c3,l
      constraintMatrixToPack.set(constraintNumber, startIndex + 10,     0.0); // c0,r follow com during single support
      constraintMatrixToPack.set(constraintNumber, startIndex + 11,     0.0); // c1,r follow com during single support
      constraintMatrixToPack.set(constraintNumber, startIndex + 12,     0.0); // c2,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 13,     0.0); // c3,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,     0.0); // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,     0.0); // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,     0.0); // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,     0.0); // a3
      
      // constraint right eCMP first constant (C_r0)
      constraintMatrixToPack.set(constraintNumber + 1, startIndex,       getECMPStart_0_FirstCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 1,   getECMPStart_0_SecondCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 4,   getECMPStart_0_ThirdCoefficient(time));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 5,   getECMPStart_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 6,   getECMPRight_0_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 7,   getECMPRight_0_SecondCoefficient(time, -1.0));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 8,   getECMPRight_0_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 9,   getECMPRight_0_FourthCoefficient(-1.0));
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 17,  0.0);
      
      // constrain left eCMP second constant (C_l1)
      constraintMatrixToPack.set(constraintNumber + 2, startIndex,       0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 1,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 4,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 5,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 6,   getECMPLeft_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 7,   getECMPLeft_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 8,   getECMPLeft_1_ThirdCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 9,   getECMPLeft_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 17,  0.0);
      
      // constrain right eCMP second constant (C_r1)
      constraintMatrixToPack.set(constraintNumber + 3, startIndex,       getECMPEnd_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 1,   getECMPEnd_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 4,   getECMPEnd_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 5,   getECMPEnd_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 6,   getECMPRight_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 7,   getECMPRight_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 8,   getECMPRight_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 9,   getECMPRight_1_FourthCoefficient(-1.0));
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 17,  0.0);
      
      /*
       * Set Objective Functions.
       */
      
      // Equals zero
      
      // Equals zero
      
      xObjectiveMatrixToPack.set(constraintNumber + 2, 0, desiredVRPStartPosition.getX());
      yObjectiveMatrixToPack.set(constraintNumber + 2, 0, desiredVRPStartPosition.getY());
      zObjectiveMatrixToPack.set(constraintNumber + 2, 0, desiredVRPStartPosition.getZ());
      
      // Equals zero
   }
   
   
   /**
    * <p> Adds 4 constraints for the left/right eCMPs during a double support state before a right footstep.</p>
    * <p> Double support states take place between two single support states. The eCMPs will be in different conditions 
    *     depending on the which footstep is prior and which one comes next. This method connects the end and start conditions
    *     to keep the eCMPs continuous. </p>
    * <p> For a right-to-left footstep, the right eCMP, eCMP<sub>r</sub>, begins at the VRP right position (VRP<sub>start</sub>) and
    *     ends at the CoM end position, x(T). The left eCMP (eCMP<sub>l</sub>) begins at the CoM, x(0), and ends at the VRP left position,
    *     VRP<sub>end</sub>. </p>
    * <p> At that footstep, this method takes care of the double support state to start a left footstep. The left eCMP would stay at the
    *     VRP during this duration and the right eCMP would follow the CoM to maintain continuity. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainECMPsForSingleRightSupportStep(double time, double omega, int sequenceId, int constraintNumber, 
                                                                     FramePoint3DReadOnly desiredVRPStartPosition,  
                                                                     DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                                                     DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj constraintMatrixToPack) {
      time = Math.min(time, sufficientlyLongTime);

      int startIndex = matrixIndex * sequenceId;
      
      /*
       * eCMP_left   = c0,l*exp(omega*t) + c1,l*exp(-omega*t) + c2,l*t + c3,l
       * eCMP_right  = c0,r*exp(omega*t) + c1,r*exp(-omega*t) + c2,r*t + c3,r
       * 
       * This function actually uses c0,l , c1,l , c0,r and c1,r
       */
      
      // constrain left eCMP first constant (C_l0)
      constraintMatrixToPack.set(constraintNumber, startIndex,       getECMPStart_0_FirstCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 1,   getECMPStart_0_SecondCoefficient(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber, startIndex + 4,   getECMPStart_0_ThirdCoefficient(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5,   getECMPStart_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 6,   getECMPLeft_0_FirstCoefficient(time, -1.0));
      constraintMatrixToPack.set(constraintNumber, startIndex + 7,   getECMPLeft_0_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 8,   getECMPLeft_0_ThirdCoefficient(-1.0));
      constraintMatrixToPack.set(constraintNumber, startIndex + 9,   getECMPLeft_0_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber, startIndex + 10,  0.0); // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 11,  0.0); // c1,r 
      constraintMatrixToPack.set(constraintNumber, startIndex + 12,  0.0); // c2,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 13,  0.0); // c3,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  0.0); // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  0.0); // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  0.0); // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  0.0); // a3
      
      // constraint right eCMP first constant (C_r0)
      constraintMatrixToPack.set(constraintNumber + 1, startIndex,       0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 1,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 4,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 5,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 6,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 7,   1.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 8,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 9,   0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 1, startIndex + 17,  0.0);
      
      // constrain left eCMP second constant (C_l1)
      constraintMatrixToPack.set(constraintNumber + 2, startIndex,       getECMPEnd_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 1,   getECMPEnd_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 4,   getECMPEnd_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 5,   getECMPEnd_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 6,   getECMPLeft_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 7,   getECMPLeft_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 8,   getECMPLeft_1_ThirdCoefficient(-1.0));
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 9,   getECMPLeft_1_FourthCoefficient());
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 2, startIndex + 17,  0.0);
      
      // constrain right eCMP second constant (C_r1)
      constraintMatrixToPack.set(constraintNumber + 3, startIndex,       0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 1,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 2,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 3,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 4,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 5,   0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 6,   getECMPRight_1_FirstCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 7,   getECMPRight_1_SecondCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 8,   getECMPRight_1_ThirdCoefficient());
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 9,   getECMPRight_1_FourthCoefficient(1.0));
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 10,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 11,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 12,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 13,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 14,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 15,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 16,  0.0);
      constraintMatrixToPack.set(constraintNumber + 3, startIndex + 17,  0.0);
      
      /*
       * Set Objective Functions.
       */
      
      // Equals zero
      
      // Equals zero
      
      // Equals zero
      
      xObjectiveMatrixToPack.set(constraintNumber + 3, 0, desiredVRPStartPosition.getX());
      yObjectiveMatrixToPack.set(constraintNumber + 3, 0, desiredVRPStartPosition.getY());
      zObjectiveMatrixToPack.set(constraintNumber + 3, 0, desiredVRPStartPosition.getZ());
   }
   
   public static void constrainComputedCoMDynamicsPosition(FramePoint3DReadOnly VRPPositionforConstraint, double time, double omega, int sequenceId, int constraintNumber,
                                                   DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                                   DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj constraintMatrixToPack) {
      
      int startIndex = matrixIndex * sequenceId;
      
      time = Math.min(time, sufficientlyLongTime);
      
      constraintMatrixToPack.set(constraintNumber, startIndex + 6,   0.0);                                                          // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 7,   0.0);                                                          // c1,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 8,   getECMPLeft_0_ThirdCoefficient(time,  1.0));                   // c2,l
      constraintMatrixToPack.set(constraintNumber, startIndex + 9,   getECMPLeft_1_FourthCoefficient(1.0));                          // c3,l
      constraintMatrixToPack.set(constraintNumber, startIndex + 10,  0.0);                                                          // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 11,  0.0);                                                          // c1,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 12,   getECMPRight_0_SecondCoefficient(time, 1.0));                 // c2,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 13,   getECMPRight_1_FourthCoefficient(1.0));                       // c3,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  -getComputedCoMDynamicsFirstCoefficient(omega, time));         // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  -getComputedCoMDynamicsSecondCoefficient(omega, time));        // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  -getComputedCoMDynamicsThirdCoefficient(time));                // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  -getComputedCoMDynamicsFourthCoefficient());                   // a3
      
//      xObjectiveMatrixToPack.set(constraintNumber, 0, VRPPositionforConstraint.getX());
//      yObjectiveMatrixToPack.set(constraintNumber, 0, VRPPositionforConstraint.getY());
//      zObjectiveMatrixToPack.set(constraintNumber, 0, VRPPositionforConstraint.getZ());
   }
   
   public static void constrainComputedCoMDynamicsVelocity(FramePoint3DReadOnly desiredVRPVelocity, double time, double omega, int sequenceId,
                                                           int constraintNumber, DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack,
                                                           DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj constraintMatrixToPack) {

      int startIndex = matrixIndex * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintNumber, startIndex + 6,   0.0); // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 7,   0.0); // c1,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 8,   1.0); // c2,l
      constraintMatrixToPack.set(constraintNumber, startIndex + 9,   0.0); // c3,l
      constraintMatrixToPack.set(constraintNumber, startIndex + 10,  0.0); // c0,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 11,  0.0); // c1,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 12,  1.0); // c2,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 13,  0.0); // c3,r
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  -getCoMVelocityFirstCoefficientTimeFunction(omega, time));   // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  -getCoMVelocitySecondCoefficientTimeFunction(omega, time));  // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  -getCoMVelocityFifthCoefficientTimeFunction());              // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  -getCoMVelocitySixthCoefficientTimeFunction());              // a3

      xObjectiveMatrixToPack.set(constraintNumber, 0, desiredVRPVelocity.getX());
      yObjectiveMatrixToPack.set(constraintNumber, 0, desiredVRPVelocity.getY());
      zObjectiveMatrixToPack.set(constraintNumber, 0, desiredVRPVelocity.getZ());
   }
   
   public static void constrainComputedCoMDCM(FramePoint3DReadOnly DCMFinalPositionforConstraint, double time, double omega, int sequenceId, int constraintNumber,
                                               DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                               DMatrixRMaj zObjectiveMatrixToPack,DMatrixRMaj constraintMatrixToPack) {
      int startIndex = matrixIndex * sequenceId;
      
      time = Math.min(time,  sufficientlyLarge);
      
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  getDCMPositionFirstCoefficientTimeFunction(omega, time));   // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  getDCMPositionSecondCoefficientTimeFunction());             // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  getDCMPositionFifthCoefficientTimeFunction(omega, time));   // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  getDCMPositionSixthCoefficientTimeFunction());              // a3
      
      xObjectiveMatrixToPack.set(constraintNumber, 0, DCMFinalPositionforConstraint.getX());
      yObjectiveMatrixToPack.set(constraintNumber, 0, DCMFinalPositionforConstraint.getY());
      zObjectiveMatrixToPack.set(constraintNumber, 0, DCMFinalPositionforConstraint.getZ());
   }
   
   public static void constrainComputedCoMPosition(FramePoint3DReadOnly centerOfMassLocationForConstraint, int sequenceId, int constraintNumber, 
                                                   DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack, 
                                                   DMatrixRMaj zObjectiveMatrixToPack,DMatrixRMaj constraintMatrixToPack) {
      int startIndex = matrixIndex * sequenceId;
      
      constraintMatrixToPack.set(constraintNumber, startIndex + 14,  1.0); // a0
      constraintMatrixToPack.set(constraintNumber, startIndex + 15,  1.0); // a1
      constraintMatrixToPack.set(constraintNumber, startIndex + 16,  0.0); // a2
      constraintMatrixToPack.set(constraintNumber, startIndex + 17,  1.0); // a3
      
      xObjectiveMatrixToPack.set(constraintNumber, 0, centerOfMassLocationForConstraint.getX());
      yObjectiveMatrixToPack.set(constraintNumber, 0, centerOfMassLocationForConstraint.getY());
      zObjectiveMatrixToPack.set(constraintNumber, 0, centerOfMassLocationForConstraint.getZ());
   }
   
   public static void constrainComputedCoMPositionContinuity(int previousSequence, int nextSequence, double omega, int constraintNumber, double previousDuration,
                                                             DMatrixRMaj constraintMatrixToPack) {
      int previousStartIndex = matrixIndex * previousSequence;
      int nextStartIndex = matrixIndex * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 14,  getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration));    
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 15,  getCoMPositionSecondCoefficientTimeFunction(omega, previousDuration));   
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 16,  getCoMPositionFifthCoefficientTimeFunction(previousDuration));           
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 17,  getCoMPositionSixthCoefficientTimeFunction());               
      
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 14,  -getCoMPositionFirstCoefficientTimeFunction(omega, 0.0));       
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 15,  -getCoMPositionSecondCoefficientTimeFunction(omega, 0.0));      
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 16,  -getCoMPositionFifthCoefficientTimeFunction(0.0));              
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 17,  -getCoMPositionSixthCoefficientTimeFunction());                  
   }
   
   public static void constrainComputedCoMVelocityContinuity(int previousSequence, int nextSequence, double omega, int constraintNumber, double previousDuration,
                                                             DMatrixRMaj constraintMatrixToPack) {
      int previousStartIndex = matrixIndex * previousSequence;
      int nextStartIndex = matrixIndex * nextSequence;
      
      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 14,  getCoMVelocityFirstCoefficientTimeFunction(omega, previousDuration));    
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 15,  getCoMVelocitySecondCoefficientTimeFunction(omega, previousDuration));   
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 16,  getCoMVelocityFifthCoefficientTimeFunction());           
      constraintMatrixToPack.set(constraintNumber, previousStartIndex + 17,  0.0);               
      
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 14,  -getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0));       
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 15,  -getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0));      
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 16,  -getCoMVelocityFifthCoefficientTimeFunction());              
      constraintMatrixToPack.set(constraintNumber, nextStartIndex + 17,  0.0);                  
   }
   
   
   /*
    *  ECMP First Coefficient which starts at the VRP for a step
    */
   /**
    * e <sup> &omega; t </sup>
    */
   public static double getECMPStart_0_FirstCoefficient(double omega, double time) {
      return Math.min(sufficientlyLarge, Math.exp(omega * time));
   }
   
   /**
    * e <sup> -&omega; t </sup>
    */
   public static double getECMPStart_0_SecondCoefficient(double omega, double time) {
      return Math.min(sufficientlyLarge, Math.exp(-omega * time));
   }
   
   /**
    * t
    */
   public static double getECMPStart_0_ThirdCoefficient(double time) {
      return Math.min(sufficientlyLarge, time);
   }
   
   /**
    * 1.0
    */
   public static double getECMPStart_0_FourthCoefficient() {
      return 1.0;
   }
   
   /*
    *  ECMP Second Coefficient which starts at the VRP for a step
    */
   /**
    * 0.0
    */
   public static double getECMPStart_1_FirstCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPStart_1_SecondCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPStart_1_ThirdCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPStart_1_FourthCoefficient() {
      return 0.0;
   }
   
   /*
    *  ECMP First Coefficient which ends at the VRP for a step
    */
   /**
    * 0.0
    */
   public static double getECMPEnd_0_FirstCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPEnd_0_SecondCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPEnd_0_ThirdCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPEnd_0_FourthCoefficient() {
      return 0.0;
   }
   
   /*
    *  ECMP second Coefficient which ends at the VRP for a step
    */
   
   /**
    * 1.0
    */
   public static double getECMPEnd_1_FirstCoefficient() {
      return 1.0;
   }
   
   /**
    * 1.0
    */
   public static double getECMPEnd_1_SecondCoefficient() {
      return 1.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPEnd_1_ThirdCoefficient() {
      return 0.0;
   }
   
   /**
    * 1.0
    */
   public static double getECMPEnd_1_FourthCoefficient() {
      return 1.0;
   }
   
   // Solving for C_0,r
   /**
    * 0.0
    */
   public static double getECMPRight_0_FirstCoefficient() {
      return 0.0;
   }
   
   /**
    * t * s
    */
   public static double getECMPRight_0_SecondCoefficient(double time, double s) {
      return Math.min(sufficientlyLarge, s*time);
   }
   
   /**
    * 0.0
    */
   public static double getECMPRight_0_ThirdCoefficient() {
      return 0.0;
   }
   
   /**
    * s
    */
   public static double getECMPRight_0_FourthCoefficient(double s) {
      return s;
   }
   
   // Solving for C_1,r
   /**
    * 0.0
    */
   public static double getECMPRight_1_FirstCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPRight_1_SecondCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPRight_1_ThirdCoefficient() {
      return 0.0;
   }
   
   /**
    * s
    */
   public static double getECMPRight_1_FourthCoefficient(double s) {
      return s;
   }

   // Solving for C_0,l
   /**
    * t * s
    */
   public static double getECMPLeft_0_FirstCoefficient(double time, double s) {
      return Math.min(sufficientlyLarge, s*time);
   }
   
   /**
    * 0.0
    */
   public static double getECMPLeft_0_SecondCoefficient() {
      return 0.0;
   }
   
   /**
    * s
    */
   public static double getECMPLeft_0_ThirdCoefficient(double s) {
      return s;
   }
   
   /**
    * 0.0
    */
   public static double getECMPLeft_0_FourthCoefficient() {
      return 0.0;
   }
   
   // Solving for C_1,l
   /**
    * 0.0
    */
   public static double getECMPLeft_1_FirstCoefficient() {
      return 0.0;
   }
   
   /**
    * 0.0
    */
   public static double getECMPLeft_1_SecondCoefficient() {
      return 0.0;
   }
   
   /**
    * s
    */
   public static double getECMPLeft_1_ThirdCoefficient(double s) {
      return s;
   }
   
   /**
    * 0.0
    */
   public static double getECMPLeft_1_FourthCoefficient() {
      return 0.0;
   }
   
   /*
    * Set Computed CoM Constraints
    */
   /**
    * e <sup> &omega; t </sup>
    */
   public static double getComputedCoMDynamicsFirstCoefficient(double omega, double time) {
      return Math.min(sufficientlyLarge, Math.exp(omega * time));
   }
   
   /**
    * e <sup> -&omega; t </sup>
    */
   public static double getComputedCoMDynamicsSecondCoefficient(double omega, double time) {
      return Math.min(sufficientlyLarge, Math.exp(-omega * time));
   }
   
   /**
    * 2t
    */
   public static double getComputedCoMDynamicsThirdCoefficient(double time) {
      return Math.min(sufficientlyLarge, 2.0*time);
   }
   
   /**
    * 2.0
    */
   public static double getComputedCoMDynamicsFourthCoefficient() {
      return 2.0;
   }

   public static double getCoMCoefficientTimeFunction(int order, int coefficient, double omega, double time)
   {
      switch (order)
      {
      case 0:
         return getCoMPositionCoefficientTimeFunction(coefficient, omega, time);
      case 1:
         return getCoMVelocityCoefficientTimeFunction(coefficient, omega, time);
      case 2:
         return getCoMAccelerationCoefficientTimeFunction(coefficient, omega, time);
      case 3:
         return getCoMJerkCoefficientTimeFunction(coefficient, omega, time);
      default:
         throw new IllegalArgumentException("The order " + order + " must be less than 3.");
      }
   }

   public static double getCoMPositionCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMPositionFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMPositionSecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMPositionThirdCoefficientTimeFunction(time);
      case 3:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMPositionFourthCoefficientTimeFunction(time);
      case 4:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMPositionFifthCoefficientTimeFunction(time);
      case 5:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMPositionSixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   public static double getCoMVelocityCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMVelocityFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMVelocitySecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMVelocityThirdCoefficientTimeFunction(time);
      case 3:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMVelocityFourthCoefficientTimeFunction(time);
      case 4:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMVelocityFifthCoefficientTimeFunction();
      case 5:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMVelocitySixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   public static double getCoMAccelerationCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMAccelerationFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMAccelerationSecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMAccelerationThirdCoefficientTimeFunction(time);
      case 3:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMAccelerationFourthCoefficientTimeFunction();
      case 4:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMAccelerationFifthCoefficientTimeFunction();
      case 5:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMAccelerationSixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   public static double getCoMJerkCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMJerkFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMJerkSecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMJerkThirdCoefficientTimeFunction();
      case 3:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMJerkFourthCoefficientTimeFunction();
      case 4:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMJerkFifthCoefficientTimeFunction();
      case 5:
         return CoMTrajectoryPlannerTools_MultipleeCMPs.getCoMJerkSixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   /**
    * e<sup>&omega; t</sup>
    */
   public static double getCoMPositionFirstCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * e<sup>-&omega; t</sup>
    */
   public static double getCoMPositionSecondCoefficientTimeFunction(double omega, double time)
   {
      return Math.exp(-omega * time);
   }

   /**
    * t<sup>3</sup>
    */
   public static double getCoMPositionThirdCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time * time * time);
   }

   /**
    * t<sup>2</sup>
    */
   public static double getCoMPositionFourthCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t
    */
   public static double getCoMPositionFifthCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getCoMPositionSixthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * &omega; e<sup>&omega; t</sup>
    */
   public static double getCoMVelocityFirstCoefficientTimeFunction(double omega, double time)
   {
      return omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega; e<sup>-&omega; t</sup>
    */
   public static double getCoMVelocitySecondCoefficientTimeFunction(double omega, double time)
   {
      return -omega * Math.exp(-omega * time);
   }

   /**
    * 3 t<sup>2</sup>
    */
   public static double getCoMVelocityThirdCoefficientTimeFunction(double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * 2 t
    */
   public static double getCoMVelocityFourthCoefficientTimeFunction(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getCoMVelocityFifthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getCoMVelocitySixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>2</sup> e<sup>&omega; t</sup>
    */
   public static double getCoMAccelerationFirstCoefficientTimeFunction(double omega, double time)
   {
      return omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * &omega;<sup>2</sup> e<sup>-&omega; t</sup>
    */
   public static double getCoMAccelerationSecondCoefficientTimeFunction(double omega, double time)
   {
      return omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6 t
    */
   public static double getCoMAccelerationThirdCoefficientTimeFunction(double time)
   {
      return 6.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 2
    */
   public static double getCoMAccelerationFourthCoefficientTimeFunction()
   {
      return 2.0;
   }

   /**
    * 0.0
    */
   public static double getCoMAccelerationFifthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getCoMAccelerationSixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>3</sup> e<sup>&omega; t</sup>
    */
   public static double getCoMJerkFirstCoefficientTimeFunction(double omega, double time)
   {
      return omega * omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega;<sup>3</sup> e<sup>-&omega; t</sup>
    */
   public static double getCoMJerkSecondCoefficientTimeFunction(double omega, double time)
   {
      return -omega * omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6.0
    */
   public static double getCoMJerkThirdCoefficientTimeFunction()
   {
      return 6.0;
   }

   /**
    * 0.0
    */
   public static double getCoMJerkFourthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getCoMJerkFifthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getCoMJerkSixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 2 e<sup>&omega; t</sup>
    */
   public static double getDCMPositionFirstCoefficientTimeFunction(double omega, double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * 0.0
    */
   public static double getDCMPositionSecondCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> + 3.0 / &omega; t<sup>2</sup>
    */
   public static double getDCMPositionThirdCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) + 3.0 / omega * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t<sup>2</sup> + 2.0 / &omega; t
    */
   public static double getDCMPositionFourthCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) + 2.0 / omega * Math.min(sufficientlyLarge, time);
   }

   /**
    * t + 1/ &omega;
    */
   public static double getDCMPositionFifthCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time) + 1.0 / omega;
   }

   /**
    * 1.0
    */
   public static double getDCMPositionSixthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getVRPPositionFirstCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getVRPPositionSecondCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> - 6.0 t / &omega;<sup>2</sup>
    */
   public static double getVRPPositionThirdCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) - 6.0 * Math.min(sufficientlyLarge, time) / (omega * omega);
   }

   /**
    * t<sup>2</sup> - 2.0 / &omega;<sup>2</sup>
    */
   public static double getVRPPositionFourthCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) - 2.0 / (omega * omega);
   }

   /**
    * t
    */
   public static double getVRPPositionFifthCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getVRPPositionSixthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getVRPVelocityFirstCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getVRPVelocitySecondCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 3 t<sup>2</sup> - 6 / &omega;<sup>2</sup>
    */
   public static double getVRPVelocityThirdCoefficientTimeFunction(double omega, double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time) - 6.0 / (omega * omega);
   }

   /**
    * 2 t
    */
   public static double getVRPVelocityFourthCoefficientTimeFunction(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getVRPVelocityFifthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getVRPVelocitySixthCoefficientTimeFunction()
   {
      return 0.0;
   }
   
   public static double getECMPPositionFirstCoefficientTimeFunction(double time) {
      return time;
   }
   
   public static double getECMPPositionSecondCoefficientTimeFunction() {
      return 1;
   }
   
   public static double getECMPVelocityFirstCoefficientTimeFunction() {
      return 1;
   }
   
   public static void constructDesiredCoMPosition(FixedFramePoint3DBasics comPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comPositionToPack.checkReferenceFrameMatch(worldFrame);
      comPositionToPack.setToZero();
      comPositionToPack.scaleAdd(getCoMPositionFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionThirdCoefficientTimeFunction(timeInPhase), thirdCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFourthCoefficientTimeFunction(timeInPhase), fourthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFifthCoefficientTimeFunction(timeInPhase), fifthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSixthCoefficientTimeFunction(), sixthCoefficient, comPositionToPack);
   }

   public static void constructDesiredCoMVelocity(FixedFrameVector3DBasics comVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comVelocityToPack.checkReferenceFrameMatch(worldFrame);
      comVelocityToPack.setToZero();
      comVelocityToPack.scaleAdd(getCoMVelocityFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityThirdCoefficientTimeFunction(timeInPhase), thirdCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFourthCoefficientTimeFunction(timeInPhase), fourthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFifthCoefficientTimeFunction(), fifthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySixthCoefficientTimeFunction(), sixthCoefficient, comVelocityToPack);
   }

   public static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics comAccelerationToPack, FramePoint3DReadOnly firstCoefficient,
                                                      FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                      FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                      FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comAccelerationToPack.checkReferenceFrameMatch(worldFrame);
      comAccelerationToPack.setToZero();
      comAccelerationToPack.scaleAdd(getCoMAccelerationFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationThirdCoefficientTimeFunction(timeInPhase), thirdCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFourthCoefficientTimeFunction(), fourthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFifthCoefficientTimeFunction(), fifthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSixthCoefficientTimeFunction(), sixthCoefficient, comAccelerationToPack);
   }
   
   public static void constructComputedCoM(FixedFramePoint3DBasics computedCoMPositionToPack, FramePoint3DReadOnly eleventhCoefficient, 
                                           FramePoint3DReadOnly twelfthCoefficient, FramePoint3DReadOnly thirteenthCoefficient, 
                                           FramePoint3DReadOnly fourteenthCoefficient, double timeInPhase, double omega) {
      computedCoMPositionToPack.checkReferenceFrameMatch(worldFrame);
      computedCoMPositionToPack.setToZero();
      computedCoMPositionToPack.scaleAdd(getCoMPositionFirstCoefficientTimeFunction(omega, timeInPhase), eleventhCoefficient, computedCoMPositionToPack); 
      computedCoMPositionToPack.scaleAdd(getCoMPositionSecondCoefficientTimeFunction(omega, timeInPhase), twelfthCoefficient, computedCoMPositionToPack);
      computedCoMPositionToPack.scaleAdd(getCoMPositionFifthCoefficientTimeFunction(timeInPhase), thirteenthCoefficient, computedCoMPositionToPack);
      computedCoMPositionToPack.scaleAdd(getCoMPositionSixthCoefficientTimeFunction(), fourteenthCoefficient, computedCoMPositionToPack);
   }

   public static void constructDesiredDCMPosition(FixedFramePoint3DBasics dcmPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      dcmPositionToPack.checkReferenceFrameMatch(worldFrame);
      dcmPositionToPack.setToZero();
      dcmPositionToPack.scaleAdd(getDCMPositionFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionSecondCoefficientTimeFunction(), secondCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionThirdCoefficientTimeFunction(omega, timeInPhase), thirdCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionFourthCoefficientTimeFunction(omega, timeInPhase), fourthCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionFifthCoefficientTimeFunction(omega, timeInPhase), fifthCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionSixthCoefficientTimeFunction(), sixthCoefficient, dcmPositionToPack);
   }

   public static void constructDesiredVRPPosition(FixedFramePoint3DBasics vrpPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      vrpPositionToPack.checkReferenceFrameMatch(worldFrame);
      vrpPositionToPack.setToZero();
      vrpPositionToPack.scaleAdd(getVRPPositionFirstCoefficientTimeFunction(), firstCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionSecondCoefficientTimeFunction(), secondCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionThirdCoefficientTimeFunction(omega, timeInPhase), thirdCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionFourthCoefficientTimeFunction(omega, timeInPhase), fourthCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionFifthCoefficientTimeFunction(timeInPhase), fifthCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionSixthCoefficientTimeFunction(), sixthCoefficient, vrpPositionToPack);
   }

   public static void constructDesiredVRPVelocity(FixedFrameVector3DBasics vrpVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      vrpVelocityToPack.checkReferenceFrameMatch(worldFrame);
      vrpVelocityToPack.setToZero();
      vrpVelocityToPack.scaleAdd(getVRPVelocityFirstCoefficientTimeFunction(), firstCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocitySecondCoefficientTimeFunction(), secondCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocityThirdCoefficientTimeFunction(omega, timeInPhase), thirdCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocityFourthCoefficientTimeFunction(timeInPhase), fourthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocityFifthCoefficientTimeFunction(), fifthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocitySixthCoefficientTimeFunction(), sixthCoefficient, vrpVelocityToPack);
   }
   
   public static void constructECMPPosition_left(FixedFramePoint3DBasics ecmpLeftPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient, 
                                                FramePoint3DReadOnly fourthCoefficient, double timeInPhase, double omega) {
      ecmpLeftPositionToPack.checkReferenceFrameMatch(worldFrame);
      ecmpLeftPositionToPack.setToZero();
      ecmpLeftPositionToPack.scaleAdd(getCoMPositionFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, ecmpLeftPositionToPack);
      ecmpLeftPositionToPack.scaleAdd(getCoMPositionSecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, ecmpLeftPositionToPack);
      ecmpLeftPositionToPack.scaleAdd(getCoMPositionFifthCoefficientTimeFunction(timeInPhase), thirdCoefficient, ecmpLeftPositionToPack);
      ecmpLeftPositionToPack.scaleAdd(getCoMPositionSixthCoefficientTimeFunction(), fourthCoefficient, ecmpLeftPositionToPack);
   }
   
   public static void constructECMPVelocity_left(FixedFramePoint3DBasics ecmpLeftVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                                 FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient, 
                                                 double timeInPhase, double omega) {
       ecmpLeftVelocityToPack.checkReferenceFrameMatch(worldFrame);
       ecmpLeftVelocityToPack.setToZero();
       ecmpLeftVelocityToPack.scaleAdd(getCoMVelocityFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, ecmpLeftVelocityToPack);
       ecmpLeftVelocityToPack.scaleAdd(getCoMVelocitySecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, ecmpLeftVelocityToPack);
       ecmpLeftVelocityToPack.scaleAdd(getCoMVelocityFifthCoefficientTimeFunction(), thirdCoefficient, ecmpLeftVelocityToPack);
    }
   
   public static void constructECMPPosition_right(FixedFramePoint3DBasics ecmpRightPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient, 
                                                  FramePoint3DReadOnly fourthCoefficient, double timeInPhase, double omega) {
      ecmpRightPositionToPack.checkReferenceFrameMatch(worldFrame);
      ecmpRightPositionToPack.setToZero();
      ecmpRightPositionToPack.scaleAdd(getCoMPositionFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, ecmpRightPositionToPack);
      ecmpRightPositionToPack.scaleAdd(getCoMPositionSecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, ecmpRightPositionToPack);
      ecmpRightPositionToPack.scaleAdd(getCoMPositionFifthCoefficientTimeFunction(timeInPhase), thirdCoefficient, ecmpRightPositionToPack);
      ecmpRightPositionToPack.scaleAdd(getCoMPositionSixthCoefficientTimeFunction(), fourthCoefficient, ecmpRightPositionToPack);
   }
   
   public static void constructECMPVelocity_right(FixedFramePoint3DBasics ecmpRightVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient, 
                                                  double timeInPhase, double omega) {
      ecmpRightVelocityToPack.checkReferenceFrameMatch(worldFrame);
      ecmpRightVelocityToPack.setToZero();
      ecmpRightVelocityToPack.scaleAdd(getCoMVelocityFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, ecmpRightVelocityToPack);
      ecmpRightVelocityToPack.scaleAdd(getCoMVelocitySecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, ecmpRightVelocityToPack);
      ecmpRightVelocityToPack.scaleAdd(getCoMVelocityFifthCoefficientTimeFunction(), thirdCoefficient, ecmpRightVelocityToPack);
   }
}
