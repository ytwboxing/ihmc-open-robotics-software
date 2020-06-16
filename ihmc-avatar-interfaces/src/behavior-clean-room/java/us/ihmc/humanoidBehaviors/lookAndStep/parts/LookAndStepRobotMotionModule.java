package us.ihmc.humanoidBehaviors.lookAndStep.parts;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;
import us.ihmc.humanoidBehaviors.lookAndStep.TypedInput;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;

import java.util.function.Supplier;

public class LookAndStepRobotMotionModule extends LookAndStepRobotMotionTask
{
   private final Field<Supplier<HumanoidRobotState>> robotStateSupplier = required();
   private Field<Supplier<LookAndStepBehavior.State>> behaviorStateSupplier = required();

   private final TypedInput<FootstepPlan> footstepPlanInput = new TypedInput<>();

   public LookAndStepRobotMotionModule(StatusLogger statusLogger)
   {
      super(statusLogger);

      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      footstepPlanInput.addCallback(data -> executor.execute(this::evaluateAndRun));
   }

   public void acceptFootstepPlan(FootstepPlan footstepPlan)
   {
      // with the gets, maybe we don't need to have validate methods

      footstepPlanInput.set(footstepPlan); // TODO: There could be data threading error here, might need to queue this data for use in the thread
   }

   private void evaluateAndRun()
   {
      validateNonChanging();

      update(footstepPlanInput.get(),
             robotStateSupplier.get().get(),
             behaviorStateSupplier.get().get());

      run();
   }

   public void setRobotStateSupplier(Supplier<HumanoidRobotState> robotStateSupplier)
   {
      this.robotStateSupplier.set(robotStateSupplier);
   }

   public void setBehaviorStateSupplier(Supplier<LookAndStepBehavior.State> behaviorStateSupplier)
   {
      this.behaviorStateSupplier.set(behaviorStateSupplier);
   }
}
