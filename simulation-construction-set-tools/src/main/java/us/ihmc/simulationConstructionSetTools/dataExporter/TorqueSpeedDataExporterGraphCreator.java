package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.awt.Color;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;

public class TorqueSpeedDataExporterGraphCreator extends DataExporterGraphCreator
{
   private static boolean DEBUG = false;
 //TODO: currently only does PinJoints
   private final List<PinJoint> pinJoints = new ArrayList<PinJoint>();

   public TorqueSpeedDataExporterGraphCreator(Robot robot, YoBuffer dataBuffer)
   {
      super(robot.getYoTime(), dataBuffer);

      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoints);
      }
   }

   public void createJointTorqueSpeedGraphs(File directory, String fileHeader, boolean createJPG, boolean createPDF)
   {
      for (PinJoint pinJoint : pinJoints)
      {
         YoBufferVariableEntry position = dataBuffer.getEntry(pinJoint.getQYoVariable());
         YoBufferVariableEntry torque = dataBuffer.getEntry(pinJoint.getTauYoVariable());
         YoBufferVariableEntry speed = dataBuffer.getEntry(pinJoint.getQDYoVariable());

         String timeLabel = "time [s]";
         String positionLabel = position.getVariableName() + " [rad]";
         String torqueLabel = torque.getVariableName() + " [Nm]";
         String speedLabel = speed.getVariableName() + " [rad/s]";

         String torqueSpeedTitle = torque.getVariableName() + "_Vs_" + speed.getVariableName();
         String torquePositionTitle = torque.getVariableName() + "_Vs_" + position.getVariableName();

         createDataVsTimeGraph(directory, fileHeader, position, createJPG, createPDF, timeLabel, positionLabel, Color.black);
         createDataVsTimeGraph(directory, fileHeader, torque, createJPG, createPDF, timeLabel, torqueLabel, Color.black);
         createDataVsTimeGraph(directory, fileHeader, speed, createJPG, createPDF, timeLabel, speedLabel, Color.black);
         createDataOneVsDataTwoGraph(directory, fileHeader, speed, torque, createJPG, createPDF, torqueSpeedTitle, speedLabel, torqueLabel, Color.black);
         createDataOneVsDataTwoGraph(directory, fileHeader, position, torque, createJPG, createPDF, torquePositionTitle, positionLabel, torqueLabel, Color.black);
      }
   }

   private void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);
      else if (DEBUG && !(joint instanceof FloatingJoint))
         PrintTools.error("Joint " + joint.getName() + " not currently handled by " + getClass().getSimpleName());

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
      }
   }
}
