package us.ihmc.comControllers.icpOptimization;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CMPRecursionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.FinalICPRecursionMultiplier;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CMPRecursionMultipliers cmpRecursionMultipliers;

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final DoubleYoVariable stanceExitCMPProjectionMultiplier;
   private final DoubleYoVariable stanceEntryCMPProjectionMultiplier;

   private final DoubleYoVariable remainingStanceExitCMPProjectionMultiplier;
   private final DoubleYoVariable remainingStanceEntryCMPProjectionMultiplier;
   private final DoubleYoVariable remainingPreviousStanceExitCMPProjectionMultiplier;

   private final DoubleYoVariable currentStateProjectionMultiplier;

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;
   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable omega;

   private final FinalICPRecursionMultiplier finalICPRecursionMultiplier;

   private final int maxNumberOfFootstepsToConsider;

   public FootstepRecursionMultiplierCalculator(DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable omega, BooleanYoVariable isInTransfer, BooleanYoVariable useTwoCMPs, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;
      this.omega = omega;
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("recursionCalculatorDoubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("recursionCalculatorSingleSupportDuration" + i, registry));
      }

      cmpRecursionMultipliers = new CMPRecursionMultipliers("new", maxNumberOfFootstepsToConsider, omega, doubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime,
            useTwoCMPs, isInTransfer, registry);

      stanceExitCMPProjectionMultiplier = new DoubleYoVariable("stanceExitCMPProjectionMultiplier", registry);
      stanceEntryCMPProjectionMultiplier = new DoubleYoVariable("stanceEntryCMPProjectionMultiplier", registry);

      remainingStanceExitCMPProjectionMultiplier = new DoubleYoVariable("remainingStanceExitCMPProjectionMultiplier", registry);
      remainingStanceEntryCMPProjectionMultiplier = new DoubleYoVariable("remainingStanceEntryCMPProjectionMultiplier", registry);
      remainingPreviousStanceExitCMPProjectionMultiplier = new DoubleYoVariable("remainingPreviousStanceExitCMPProjectionMultiplier", registry);

      currentStateProjectionMultiplier = new DoubleYoVariable("currentSTateProjectionMultiplier", registry);

      finalICPRecursionMultiplier = new FinalICPRecursionMultiplier(registry, omega, doubleSupportSplitFraction, isInTransfer);

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
      cmpRecursionMultipliers.reset();
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

      cmpRecursionMultipliers.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations);

      stanceExitCMPProjectionMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeToFinish));
      stanceEntryCMPProjectionMultiplier.set(0.0);

      finalICPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations);
   }

   private void computeRecursionMultipliersForTwoCMPs(int numberOfStepsToConsider, boolean isInTransfer)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnInitialDoubleSupportUpcoming = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      if (isInTransfer)
      {
         stanceExitCMPProjectionMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeSpentOnInitialDoubleSupportUpcoming));
         stanceEntryCMPProjectionMultiplier.set(0.0);
      }
      else
      {
         double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;
         double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * firstStepTime;

         double projectionTime = totalTimeSpentOnEntryCMP + timeSpentOnEndDoubleSupportCurrent;
         double multiplier = Math.exp(-omega.getDoubleValue() * projectionTime);

         stanceExitCMPProjectionMultiplier.set(multiplier * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP)));
         stanceEntryCMPProjectionMultiplier.set(1.0 - multiplier);
      }
   }

   public void computeRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      computeStanceFootRemainingProjectionMultipliers(timeRemaining, useTwoCMPs, isInTransfer, isInTransferEntry);
      computeCurrentStateProjection(timeRemaining);
   }

   private void computeCurrentStateProjection(double timeRemaining)
   {
      currentStateProjectionMultiplier.set(Math.exp(omega.getDoubleValue() * timeRemaining));
   }

   private void computeStanceFootRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
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

   public double getCMPRecursionExitMultiplier(int footstepIndex)
   {
      return cmpRecursionMultipliers.getExitMultiplier(footstepIndex);
   }

   public double getCMPRecursionEntryMultiplier(int footstepIndex)
   {
      return cmpRecursionMultipliers.getEntryMultiplier(footstepIndex);
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

   public double getCurrentStateProjectionMultiplier()
   {
      return currentStateProjectionMultiplier.getDoubleValue();
   }
}
