package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoFunctionGeneratorVisualizer implements RobotController
{
   private YoRegistry registry = new YoRegistry("YoFunGenViz");
   
   private YoDouble valueCheck;
   
   private YoFunctionGenerator yoFunctionGenerator;
   private YoDouble time;
   
   private YoDouble resetTime;
   private YoDouble maxSweepFreq;
   private YoDouble amplitude;

   private YoBoolean hasBeenReset;

   private final YoEnum<YoFunctionGeneratorMode> mode;

   
   public YoFunctionGeneratorVisualizer(YoFunctionGenerator yoFunctionGenerator)
   {
      this.yoFunctionGenerator = yoFunctionGenerator;
      
      mode = new YoEnum<>("Mode", registry, YoFunctionGeneratorMode.class);
      
      resetTime = new YoDouble("resetTime", registry);
      
      resetTime.set(20.0);
      
      maxSweepFreq = new YoDouble("maxSweepFreq", registry);
      maxSweepFreq.set(60.0);
      
      amplitude = new YoDouble("amplitude", registry);
      amplitude.set(1.0);
      
      valueCheck = new YoDouble("valueCheck", registry);
      
      hasBeenReset = new YoBoolean("hasBeenReset", registry);
      hasBeenReset.set(true);
   }
   
   @Override
   public void initialize()
   {
      
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }
   
   public void setTimeVariable(YoDouble time)
   {
      this.time = time;
   }
   
   
   @Override
   public void doControl()
   {
      if (!hasBeenReset.getBooleanValue() && !mode.getEnumValue().equals(YoFunctionGeneratorMode.OFF) && yoFunctionGenerator.getMode().equals(YoFunctionGeneratorMode.OFF))
      {
         mode.set(YoFunctionGeneratorMode.OFF);
         hasBeenReset.set(true);
      }
      
      if (!yoFunctionGenerator.getMode().equals(YoFunctionGeneratorMode.OFF))
         hasBeenReset.set(false);
      
      yoFunctionGenerator.setMode(mode.getEnumValue());
      yoFunctionGenerator.setResetTime(resetTime.getDoubleValue());
      yoFunctionGenerator.setChirpFrequencyMaxHz(maxSweepFreq.getDoubleValue());
      yoFunctionGenerator.setAmplitude(amplitude.getDoubleValue());
      
      valueCheck.set(this.yoFunctionGenerator.getValue(time.getDoubleValue()));
      
      try
      {
         Thread.sleep(1);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }
}


