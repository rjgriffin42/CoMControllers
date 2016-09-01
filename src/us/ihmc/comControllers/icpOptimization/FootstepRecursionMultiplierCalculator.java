package us.ihmc.comControllers.icpOptimization;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CMPRecursionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.FinalICPRecursionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.RemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.StanceCMPProjectionMultipliers;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final DoubleYoVariable currentStateProjectionMultiplier;

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;
   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable omega;

   private final FinalICPRecursionMultiplier finalICPRecursionMultiplier;
   private final CMPRecursionMultipliers cmpRecursionMultipliers;
   private final StanceCMPProjectionMultipliers stanceCMPProjectionMultipliers;
   private final RemainingStanceCMPProjectionMultipliers remainingStanceCMPProjectionMultipliers;

   private final int maxNumberOfFootstepsToConsider;

   public FootstepRecursionMultiplierCalculator(DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable omega, BooleanYoVariable isInTransfer, BooleanYoVariable isInTransferEntry, BooleanYoVariable useTwoCMPs, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
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

      cmpRecursionMultipliers = new CMPRecursionMultipliers("", maxNumberOfFootstepsToConsider, omega, doubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime,
            useTwoCMPs, isInTransfer, registry);
      stanceCMPProjectionMultipliers = new StanceCMPProjectionMultipliers("", omega, doubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime, useTwoCMPs, isInTransfer, registry);
      remainingStanceCMPProjectionMultipliers = new RemainingStanceCMPProjectionMultipliers("", omega, doubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime,
            useTwoCMPs, isInTransfer, isInTransferEntry, registry);

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
      stanceCMPProjectionMultipliers.reset();
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, boolean isInTransfer, boolean useTwoCMPs)
   {
      reset();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      finalICPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations);
      stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations);
      cmpRecursionMultipliers.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations);
   }

   public void computeRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      computeCurrentStateProjection(timeRemaining);

      remainingStanceCMPProjectionMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations);
   }

   private void computeCurrentStateProjection(double timeRemaining)
   {
      currentStateProjectionMultiplier.set(Math.exp(omega.getDoubleValue() * timeRemaining));
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
      return stanceCMPProjectionMultipliers.getExitMultiplier();
   }

   public double getStanceEntryCMPProjectionMultiplier()
   {
      return stanceCMPProjectionMultipliers.getEntryMultiplier();
   }

   public double getRemainingStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getExitMultiplier();
   }

   public double getRemainingStanceEntryCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getEntryMultiplier();
   }

   public double getRemainingPreviousStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getPreviousExitMultiplier();
   }

   public double getCurrentStateProjectionMultiplier()
   {
      return currentStateProjectionMultiplier.getDoubleValue();
   }
}
