package us.ihmc.comControllers.icpOptimization;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> cmpRecursionEntryMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> cmpRecursionExitMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final DoubleYoVariable stanceExitCMPProjectionMultiplier;
   private final DoubleYoVariable stanceEntryCMPProjectionMultiplier;

   private final DoubleYoVariable remainingStanceExitCMPProjectionMultiplier;
   private final DoubleYoVariable remainingStanceEntryCMPProjectionMultiplier;
   private final DoubleYoVariable remainingPreviousStanceExitCMPProjectionMultiplier;

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;
   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable omega;
   private final DoubleYoVariable finalICPRecursionMultiplier;

   private final int maxNumberOfFootstepsToConsider;

   public FootstepRecursionMultiplierCalculator(DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable omega, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;
      this.omega = omega;
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         cmpRecursionEntryMultipliers.add(new DoubleYoVariable("cmpRecursionEntryMultiplier" + i, registry));
         cmpRecursionExitMultipliers.add(new DoubleYoVariable("cmpRecursionExitMultiplier" + i, registry));

         doubleSupportDurations.add(new DoubleYoVariable("recursionCalculatorDoubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("recursionCalculatorSingleSupportDuration" + i, registry));
      }

      stanceExitCMPProjectionMultiplier = new DoubleYoVariable("stanceExitCMPProjectionMultiplier", registry);
      stanceEntryCMPProjectionMultiplier = new DoubleYoVariable("stanceEntryCMPProjectionMultiplier", registry);

      remainingStanceExitCMPProjectionMultiplier = new DoubleYoVariable("remainingStanceExitCMPProjectionMultiplier", registry);
      remainingStanceEntryCMPProjectionMultiplier = new DoubleYoVariable("remainingStanceEntryCMPProjectionMultiplier", registry);
      remainingPreviousStanceExitCMPProjectionMultiplier = new DoubleYoVariable("remainingPreviousStanceExitCMPProjectionMultiplier", registry);

      finalICPRecursionMultiplier = new DoubleYoVariable("finalICPRecursionMultiplier", registry);

      parentRegistry.addChild(registry);
   }

   public void resetTimes()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         doubleSupportDurations.get(i).set(0.0);
         singleSupportDurations.get(i).set(0.0);
      }
   }

   public void submitTimes(int footstepIndex, double doubleSupportDuration, double singleSupportDuration)
   {
      doubleSupportDurations.get(footstepIndex).set(doubleSupportDuration);
      singleSupportDurations.get(footstepIndex).set(singleSupportDuration);
   }

   public void reset()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         cmpRecursionEntryMultipliers.get(i).set(0.0);
         cmpRecursionExitMultipliers.get(i).set(0.0);
      }
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, boolean isInTransfer, boolean useTwoCMPs)
   {
      reset();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      if (useTwoCMPs)
         computeRecursionMultipliersForTwoCMPs(numberOfStepsToConsider, isInTransfer);
      else
         computeRecursionMultipliersForOneCMP(numberOfStepsToConsider, isInTransfer);
   }

   private void computeRecursionMultipliersForOneCMP(int numberOfStepsToConsider, boolean isInTransfer)
   {
      double timeToFinish = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();

      if (isInTransfer)
         timeToFinish += singleSupportDurations.get(0).getDoubleValue();

      double recursionTime = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration = doubleSupportDurations.get(i + 1).getDoubleValue() + singleSupportDurations.get(i + 1).getDoubleValue();
         double stepRecursion = 1.0 - Math.exp(-omega.getDoubleValue() * steppingDuration);

         double previousStepDuration = 0.0;
         if (i > 0)
            previousStepDuration = doubleSupportDurations.get(i).getDoubleValue() + singleSupportDurations.get(i).getDoubleValue();

         recursionTime += previousStepDuration;
         cmpRecursionExitMultipliers.get(i).set(Math.exp(-omega.getDoubleValue() * recursionTime) * stepRecursion);
         cmpRecursionEntryMultipliers.get(i).set(0.0);
      }

      stanceExitCMPProjectionMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeToFinish));
      stanceEntryCMPProjectionMultiplier.set(0.0);

      double totalTimeForFinalICPRecursion = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
         totalTimeForFinalICPRecursion += singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();

      computeFinalICPRecursionMultiplier(totalTimeForFinalICPRecursion, omega.getDoubleValue());
   }

   private void computeRecursionMultipliersForTwoCMPs(int numberOfStepsToConsider, boolean isInTransfer)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnInitialDoubleSupportUpcoming = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double timeToFinish;
      if (isInTransfer)
      {
         timeToFinish = -timeSpentOnEndDoubleSupportCurrent + firstStepTime;

         stanceExitCMPProjectionMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeSpentOnInitialDoubleSupportUpcoming));
         stanceEntryCMPProjectionMultiplier.set(0.0);
      }
      else
      {
         timeToFinish = timeSpentOnInitialDoubleSupportUpcoming;

         double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;
         double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * firstStepTime;

         double projectionTime = totalTimeSpentOnEntryCMP + timeSpentOnEndDoubleSupportCurrent;
         double multiplier = Math.exp(-omega.getDoubleValue() * projectionTime);

         stanceExitCMPProjectionMultiplier.set(multiplier * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP)));
         stanceEntryCMPProjectionMultiplier.set(1.0 - multiplier);
      }

      double recursionTime = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration = singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();
         double previousStepDuration = 0.0;
         if (i > 0)
            previousStepDuration = doubleSupportDurations.get(i).getDoubleValue() + singleSupportDurations.get(i).getDoubleValue();

         recursionTime += previousStepDuration;
         double multiplier = Math.exp(-omega.getDoubleValue() * recursionTime);

         double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * steppingDuration;
         double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;

         double exitRecursion = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         double entryRecursion = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);

         cmpRecursionEntryMultipliers.get(i).set(multiplier * entryRecursion);
         cmpRecursionExitMultipliers.get(i).set(multiplier * exitRecursion);
      }
   }

   public void computeStanceFootRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double remainingProjection = Math.exp(omega.getDoubleValue() * timeRemaining);
      double endOfSupportProjection = Math.exp(omega.getDoubleValue() * timeSpentOnEndDoubleSupportCurrent);

      if (isInTransfer)
      {
         if (isInTransferEntry)
         {
            if (useTwoCMPs)
            {
               remainingStanceExitCMPProjectionMultiplier.set(0.0);
               remainingStanceEntryCMPProjectionMultiplier.set(endOfSupportProjection - 1.0);
            }
            else
            {
               remainingStanceExitCMPProjectionMultiplier.set(endOfSupportProjection - 1.0);
               remainingStanceEntryCMPProjectionMultiplier.set(0.0);
            }
            remainingPreviousStanceExitCMPProjectionMultiplier.set(remainingProjection - endOfSupportProjection);
         }
         else
         {
            if (useTwoCMPs)
            {
               remainingStanceExitCMPProjectionMultiplier.set(0.0);
               remainingStanceEntryCMPProjectionMultiplier.set(remainingProjection - 1.0);
            }
            else
            {
               remainingStanceExitCMPProjectionMultiplier.set(remainingProjection - 1.0);
               remainingStanceEntryCMPProjectionMultiplier.set(0.0);
            }
            remainingPreviousStanceExitCMPProjectionMultiplier.set(0.0);
         }
      }
      else
      {
         remainingStanceExitCMPProjectionMultiplier.set(remainingProjection - 1.0);
         remainingStanceEntryCMPProjectionMultiplier.set(0.0);
         remainingPreviousStanceExitCMPProjectionMultiplier.set(0.0);

      }
   }

   private void computeFinalICPRecursionMultiplier(double totalTime, double omega)
   {
      finalICPRecursionMultiplier.set(Math.exp(-omega * totalTime));
   }

   public double getCMPRecursionExitMultiplier(int footstepIndex, boolean useTwoCMPs)
   {
      if (useTwoCMPs)
         throw new RuntimeException("Trying to get two CMP recursion multiplier.");

      return cmpRecursionExitMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getCMPRecursionEntryMultiplier(int footstepIndex, boolean useTwoCMPs)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return cmpRecursionEntryMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getFinalICPRecursionMultiplier()
   {
      return finalICPRecursionMultiplier.getDoubleValue();
   }

   public double getStanceExitCMPProjectionMultiplier()
   {
      return stanceExitCMPProjectionMultiplier.getDoubleValue();
   }

   public double getStanceEntryCMPProjectionMultiplier()
   {
      return stanceEntryCMPProjectionMultiplier.getDoubleValue();
   }

   public double getRemainingStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceExitCMPProjectionMultiplier.getDoubleValue();
   }

   public double getRemainingStanceEntryCMPProjectionMultiplier()
   {
      return remainingStanceEntryCMPProjectionMultiplier.getDoubleValue();
   }

   public double getRemainingPreviousStanceExitCMPProjectionMultiplier()
   {
      return remainingPreviousStanceExitCMPProjectionMultiplier.getDoubleValue();
   }
}
