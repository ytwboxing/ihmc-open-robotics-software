package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class ConstantPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final YoFramePoint3D position;
   private final YoDouble finalTime;
   private final YoDouble time;
   private final PositionProvider positionProvider;

   public ConstantPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider positionProvider, double finalTime,
           YoVariableRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.positionProvider = positionProvider;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.position = new YoFramePoint3D("position", referenceFrame, registry);
      this.finalTime = new YoDouble("finalTime", registry);
      this.time = new YoDouble("time", registry);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
      positionProvider.getPosition(position);
   }

   public void compute(double time)
   {
      this.time.set(time);
   }

   public boolean isDone()
   {
      return time.getDoubleValue() > finalTime.getDoubleValue();
   }

   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.set(position);
   }

   public void getVelocity(FixedFrameVector3DBasics velocityToPack)
   {
      velocityToPack.checkReferenceFrameMatch(position);
      velocityToPack.setToZero();
   }

   public void getAcceleration(FixedFrameVector3DBasics accelerationToPack)
   {
      accelerationToPack.checkReferenceFrameMatch(position);
      accelerationToPack.setToZero();
   }

   public void showVisualization()
   {
   }

   public void hideVisualization()
   {
   }
}
