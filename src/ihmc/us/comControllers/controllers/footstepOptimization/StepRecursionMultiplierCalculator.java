package ihmc.us.comControllers.controllers.footstepOptimization;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class StepRecursionMultiplierCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> oneCMPRecursionMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> twoCMPRecursionEntryMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> twoCMPRecursionExitMultipliers = new ArrayList<>();

   private final DoubleYoVariable finalICPRecursionMultiplier;

   private final int maxNumberOfFootstepsToConsider;

   public StepRecursionMultiplierCalculator(int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         oneCMPRecursionMultipliers.add(new DoubleYoVariable("oneCMPRecursionMultiplier" + i, registry));
         twoCMPRecursionEntryMultipliers.add(new DoubleYoVariable("twoCMPRecursionEntryMultiplier" + i, registry));
         twoCMPRecursionExitMultipliers.add(new DoubleYoVariable("twoCMPRecursionExitMultiplier" + i, registry));
      }

      finalICPRecursionMultiplier = new DoubleYoVariable("finalICPRecursionMultiplier", registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         oneCMPRecursionMultipliers.get(i).set(0.0);
         twoCMPRecursionEntryMultipliers.get(i).set(0.0);
         twoCMPRecursionExitMultipliers.get(i).set(0.0);
      }

   }

   public void computeRecursionMultipliers(double totalTimeSpentOnExitCMP, double totalTimeSpentOnEntryCMP, double omega, int numberOfStepsToConsider, boolean useTwoCMPS)
   {
      if (!useTwoCMPS)
         throw new RuntimeException("Trying to compute two CMP recursion multipliers.");

      double steppingDuration = totalTimeSpentOnEntryCMP + totalTimeSpentOnExitCMP;
      double exitRecursion = 1.0 - Math.exp(-omega * totalTimeSpentOnExitCMP);
      double entryRecursion = 1.0 - Math.exp(-omega * totalTimeSpentOnEntryCMP);

      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double multiplier = Math.exp(-omega * (i - 1) * steppingDuration);
         twoCMPRecursionEntryMultipliers.get(i).set(multiplier * entryRecursion);
         twoCMPRecursionExitMultipliers.get(i).set(multiplier * exitRecursion);
      }

      computeFinalICPRecursionMultiplier(steppingDuration, omega, numberOfStepsToConsider);
   }

   public void computeRecursionMultipliers(double steppingDuration, int numberOfStepsToConsider, double omega, boolean useTwoCMPS)
   {
      if (useTwoCMPS)
         throw new RuntimeException("Trying to compute one CMP recursion multipliers.");

      double stepRecursion = 1.0 - Math.exp(-omega * steppingDuration);

      for (int i = 0; i < numberOfStepsToConsider; i++)
         oneCMPRecursionMultipliers.get(i).set(Math.exp(-omega * (i - 1) * steppingDuration) * stepRecursion);

      computeFinalICPRecursionMultiplier(steppingDuration, omega, numberOfStepsToConsider);
   }

   private void computeFinalICPRecursionMultiplier(double steppingDuration, double omega, int numberOfStepsToConsider)
   {
      double totalTime = steppingDuration * numberOfStepsToConsider;
      finalICPRecursionMultiplier.set(Math.exp(-omega * totalTime));
   }

   public double getOneCMPRecursionMultiplier(int footstepIndex, boolean useTwoCMPS)
   {
      if (useTwoCMPS)
         throw new RuntimeException("Trying to get two CMP recursion multiplier.");

      return oneCMPRecursionMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getTwoCMPRecursionEntryMultiplier(int footstepIndex, boolean useTwoCMPS)
   {
      if (!useTwoCMPS)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPRecursionEntryMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getTwoCMPRecursionExitMultiplier(int footstepIndex, boolean useTwoCMPS)
   {
      if (!useTwoCMPS)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPRecursionExitMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getFinalICPRecursionMultiplier()
   {
      return finalICPRecursionMultiplier.getDoubleValue();
   }

}


