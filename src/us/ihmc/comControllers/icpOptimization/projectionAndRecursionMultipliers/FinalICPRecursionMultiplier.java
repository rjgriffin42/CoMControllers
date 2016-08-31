package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FinalICPRecursionMultiplier extends DoubleYoVariable
{
   private static final String namePrefix = "finalICPRecursionMultiplier";

   private final DoubleYoVariable omega;

   private final BooleanYoVariable isInTransfer;

   private final DoubleYoVariable doubleSupportSplitFraction;

   public FinalICPRecursionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitFraction,
         BooleanYoVariable isInTransfer)
   {
      this(namePrefix, registry, omega, doubleSupportSplitFraction, isInTransfer);
   }

   public FinalICPRecursionMultiplier(String name, YoVariableRegistry registry, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitFraction,
         BooleanYoVariable isInTransfer)
   {
      super(name, registry);

      this.omega = omega;

      if (doubleSupportSplitFraction == null)
         this.doubleSupportSplitFraction = new DoubleYoVariable(namePrefix + "_DoubleSupportSplitFraction", registry);
      else
         this.doubleSupportSplitFraction = doubleSupportSplitFraction;

      if (isInTransfer == null)
         this.isInTransfer = new BooleanYoVariable(namePrefix + "_IsInTransfer", registry);
      else
         this.isInTransfer = isInTransfer;
   }

   public void compute(int numberOfStepsToConsider, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations)
   {
      if (numberOfStepsToConsider > doubleSupportDurations.size())
         throw new RuntimeException("Double Support Durations list is not long enough");
      if (numberOfStepsToConsider > singleSupportDurations.size())
         throw new RuntimeException("Single Support Durations list is not long enough");

      double timeToFinish = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (isInTransfer.getBooleanValue())
         timeToFinish += singleSupportDurations.get(0).getDoubleValue();

      double totalTimeForFinalICPRecursion = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
         totalTimeForFinalICPRecursion += singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();

      this.set(Math.exp(-omega.getDoubleValue() * totalTimeForFinalICPRecursion));
   }
}
