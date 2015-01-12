package us.ihmc.steppr.hardware.command;

import java.nio.ByteBuffer;

import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class StepprActuatorCommand
{
   private final YoVariableRegistry registry;
   private final BooleanYoVariable enabled;
   private final DoubleYoVariable tauDesired;
   private final DoubleYoVariable currentDesired;
   private final DoubleYoVariable damping;
   
   StepprActuator actuator;
   
   public StepprActuatorCommand(String name, StepprActuator actuator, YoVariableRegistry parentRegistry)
   {
      this.actuator = actuator;
      this.registry = new YoVariableRegistry(name);

      this.enabled = new BooleanYoVariable(name + "Enabled", registry);
      this.tauDesired = new DoubleYoVariable(name + "TauDesired", registry);
      this.damping = new DoubleYoVariable(name + "Damping", registry);
      this.currentDesired = new DoubleYoVariable(name+"CurrentDesired", registry);
      
      parentRegistry.addChild(registry);
   }
   
   public abstract void update();
   
   

   public void write(ByteBuffer target, int controlID)
   {
      if(enabled.getBooleanValue())
      {
         target.put((byte) 3);
         target.putFloat((float) (currentDesired.getDoubleValue()));
         target.putFloat((float) (damping.getDoubleValue() / actuator.getKt()));
         target.putFloat(0f);
         target.putInt(controlID);
      }
      else
      {
         target.put((byte) 0);
         target.putFloat(0f);
         target.putFloat(0f);
         target.putFloat(0f);
         target.putInt(controlID);
      }
   }

   public void enable()
   {
      enabled.set(true);
   }

   public void disable()
   {
      enabled.set(false);
   }
   
   protected double getTauDesired()
   {
	   return tauDesired.getDoubleValue();
   }
   
   protected void setTauDesired(double tau)
   {
      tauDesired.set(tau);
      this.currentDesired.set(tau/actuator.getKt());
   }
   
   protected void setDamping(double damping)
   {
      this.damping.set(damping);
   }
}
