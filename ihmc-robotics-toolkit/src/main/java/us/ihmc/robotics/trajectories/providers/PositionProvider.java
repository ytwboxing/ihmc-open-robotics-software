package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;

public interface PositionProvider
{
   void getPosition(FixedFramePoint3DBasics positionToPack);
}
