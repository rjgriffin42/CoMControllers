package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public abstract class CurrentStateProjectionMultiplier extends DoubleYoVariable
{
   private static final String name = "CurrentStateProjectionMultiplier";

   protected final DoubleYoVariable omega;

   public CurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega)
   {
      this("", registry, omega);
   }

   public CurrentStateProjectionMultiplier(String namePrefix, YoVariableRegistry registry, DoubleYoVariable omega)
   {
      super(namePrefix + name, registry);

      this.omega = omega;
   }

   public abstract void compute(double timeRemaining);

   public abstract void reset();
}
