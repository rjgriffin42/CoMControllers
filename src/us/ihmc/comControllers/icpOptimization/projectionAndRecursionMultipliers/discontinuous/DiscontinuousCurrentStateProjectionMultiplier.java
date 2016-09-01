package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.discontinuous;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CurrentStateProjectionMultiplier;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DiscontinuousCurrentStateProjectionMultiplier extends CurrentStateProjectionMultiplier
{
   public DiscontinuousCurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega)
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
