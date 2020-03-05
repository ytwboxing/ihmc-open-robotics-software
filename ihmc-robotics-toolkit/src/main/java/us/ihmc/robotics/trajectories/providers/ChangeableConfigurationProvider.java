package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;

/**
* @author twan
*         Date: 5/27/13
*/
public class ChangeableConfigurationProvider implements SE3ConfigurationProvider
{
   private final FramePose3D configuration;

   public ChangeableConfigurationProvider()
   {
      configuration = new FramePose3D();
   }

   public ChangeableConfigurationProvider(FramePose3D initialConfiguration)
   {
      configuration = new FramePose3D(initialConfiguration);
   }

   public void get(FramePose3D framePose)
   {
      framePose.setIncludingFrame(configuration);
   }

   @Override
   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.set(configuration.getPosition());
   }

   @Override
   public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.set(configuration.getOrientation());
   }

   public void set(FramePose3D newPose)
   {
      configuration.setIncludingFrame(newPose);
   }

}
