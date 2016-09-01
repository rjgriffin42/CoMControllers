package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CurrentStateProjectionMultiplier;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ContinuousCurrentStateProjectionMultiplier extends CurrentStateProjectionMultiplier
{
   public ContinuousCurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega)
   {
      super(registry, omega);
   }

   public void compute(double timeRemaining)
   {
      this.set(Math.exp(omega.getDoubleValue() * timeRemaining));
   }

   public void reset()
   {
      this.set(0.0);
   }
}
