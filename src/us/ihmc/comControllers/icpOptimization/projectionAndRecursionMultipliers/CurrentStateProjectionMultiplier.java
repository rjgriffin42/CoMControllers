package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public abstract class CurrentStateProjectionMultiplier extends DoubleYoVariable
{
   private static final String name = "CurrentStateProjectionMultiplier";

   public CurrentStateProjectionMultiplier(YoVariableRegistry registry)
   {
      this("", registry);
   }

   public CurrentStateProjectionMultiplier(String namePrefix, YoVariableRegistry registry)
   {
      super(namePrefix + name, registry);
   }

   public abstract void compute(double timeRemaining);

   public abstract void reset();
}
