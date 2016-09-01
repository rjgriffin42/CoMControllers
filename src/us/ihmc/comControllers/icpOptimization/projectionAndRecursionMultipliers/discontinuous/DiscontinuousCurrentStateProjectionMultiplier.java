package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.discontinuous;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CurrentStateProjectionMultiplier;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class DiscontinuousCurrentStateProjectionMultiplier extends CurrentStateProjectionMultiplier
{
   private final DoubleYoVariable omega;

   public DiscontinuousCurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega)
   {
      super(registry);

      this.omega = omega;
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                       boolean useTwoCMPs, boolean isInTransfer)
   {
      this.set(Math.exp(omega.getDoubleValue() * timeRemaining));
   }

   public void reset()
   {
      this.set(0.0);
   }
}
