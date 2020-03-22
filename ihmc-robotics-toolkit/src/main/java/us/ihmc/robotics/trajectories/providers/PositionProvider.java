package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface PositionProvider extends ReferenceFrameHolder
{
   default void getPosition(FramePoint3DBasics positionToPack)
   {
      positionToPack.setReferenceFrame(this.getReferenceFrame());
      getPosition((FixedFramePoint3DBasics) positionToPack);
   }

   void getPosition(FixedFramePoint3DBasics positionToPack);
}
