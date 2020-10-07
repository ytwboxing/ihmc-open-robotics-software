package us.ihmc.atlas;

import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.DomainFactory;

public class AtlasFootstepPlanningToolbox
{
   public static void main(String[] args)
   {
      FootstepPlanningModuleLauncher.createModule(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS),
                                                  DomainFactory.PubSubImplementation.FAST_RTPS);

      ThreadTools.sleepForever();
   }
}
