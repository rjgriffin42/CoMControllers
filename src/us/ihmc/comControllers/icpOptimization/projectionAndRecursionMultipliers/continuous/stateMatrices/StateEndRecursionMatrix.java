package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class StateEndRecursionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;

   public StateEndRecursionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer)
   {
      double stateDuration;
      if (isInTransfer)
      {
         stateDuration = doubleSupportDurations.get(0).getDoubleValue();
      }
      else
      {
         if (useTwoCMPs)
         {
            double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
            double currentEndOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();
            double stepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

            stateDuration = upcomingInitialDoubleSupportDuration + currentEndOfDoubleSupportDuration - stepDuration;
         }
         else
         {
            stateDuration = singleSupportDurations.get(0).getDoubleValue();
         }
      }

      this.compute(stateDuration);
   }

   public void compute(double stateDuration)
   {
      double stateRecursion = Math.exp(-omega.getDoubleValue() * stateDuration);

      set(0, 0, stateRecursion);
      set(1, 0, omega.getDoubleValue() * stateRecursion);
      set(2, 0, 1.0);
      set(3, 0, omega.getDoubleValue());
   }
}
