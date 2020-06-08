package us.ihmc.robotEnvironmentAwareness.geometry;

public class VariancePredictor
{
   private long numberOfSamples;
   private double firstMoment;
   private double secondMoment;

   public void clear()
   {
      firstMoment = Double.NaN;
      secondMoment = Double.NaN;
      numberOfSamples = 0;
   }

   public double predictIncrement(final double value)
   {
      double firstMoment = this.firstMoment;
      double secondMoment = this.secondMoment;
      if (numberOfSamples < 1)
         firstMoment = secondMoment = 0.0;

      if (numberOfSamples == 0)
         firstMoment = 0.0;

      double dev = value - firstMoment;
      double nDev = dev / (numberOfSamples + 1);

      secondMoment += ((double) numberOfSamples) * dev * nDev;
      return secondMoment / (numberOfSamples + 1);
   }

   public void increment(final double value)
   {
      if (numberOfSamples < 1)
      {
         firstMoment = secondMoment = 0.0;
      }

      if (numberOfSamples == 0)
      {
         firstMoment = 0.0;
      }
      numberOfSamples++;
      double dev = value - firstMoment;
      double nDev = dev / numberOfSamples;
      firstMoment += nDev;

      secondMoment += ((double) numberOfSamples - 1) * dev * nDev;
   }

   public double getVariance()
   {
      if (numberOfSamples == 0)
      {
         return Double.NaN;
      }
      else if (numberOfSamples == 1)
      {
         return 0d;
      }
      else
      {
         return secondMoment / (numberOfSamples);
      }
   }

   public long getNumberOfSamples()
   {
      return numberOfSamples;
   }
}
