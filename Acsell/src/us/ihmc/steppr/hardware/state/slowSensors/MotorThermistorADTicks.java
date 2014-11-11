package us.ihmc.steppr.hardware.state.slowSensors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class MotorThermistorADTicks implements StepprSlowSensor
{
   private final IntegerYoVariable motorTemperature;
   
   public MotorThermistorADTicks(String name, YoVariableRegistry parentRegistry)
   {
      motorTemperature = new IntegerYoVariable(name + "MotorThermisterADTicks", parentRegistry);
   }

   @Override
   public void update(int value)
   {
      motorTemperature.set(value);
   }
   
}
