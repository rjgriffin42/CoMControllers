package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class CurrentStateProjectionMultiplier extends DoubleYoVariable
{
   private static final String name = "CurrentStateProjectionMultiplier";

   private final DoubleYoVariable omega;

   public CurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega)
   {
      this("", registry, omega);
   }

   public CurrentStateProjectionMultiplier(String namePrefix, YoVariableRegistry registry, DoubleYoVariable omega)
   {
      super(namePrefix + name, registry);

      this.omega = omega;
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
