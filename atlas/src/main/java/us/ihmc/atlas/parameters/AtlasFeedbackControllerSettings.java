package us.ihmc.atlas.parameters;

import java.util.Arrays;
import java.util.List;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;

public class AtlasFeedbackControllerSettings implements FeedbackControllerSettings
{
   private final AtlasJointMap jointMap;
   private final RobotTarget target;

   public AtlasFeedbackControllerSettings(AtlasJointMap jointMap, RobotTarget target)
   {
      this.jointMap = jointMap;
      this.target = target;
   }

   @Override
   public boolean enableIntegralTerm()
   {
      return false; // Saves about 130 YoVariables.
   }

   @Override
   public List<GroupParameter<Double>> getErrorVelocityFilterBreakFrequencies()
   {
      if (target == RobotTarget.SCS)
         return null;

      GroupParameter<Double> pelvisGroup = new GroupParameter<>(jointMap.getPelvisName(), 30.0);
      GroupParameter<Double> chestGroup = new GroupParameter<>(jointMap.getChestName(), 30.0);

      return Arrays.asList(pelvisGroup, chestGroup);
   }
}
