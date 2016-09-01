package us.ihmc.comControllers.icpOptimization;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.*;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.ContinuousCurrentStateProjectionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.ContinuousRemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.discontinuous.DiscontinuousCurrentStateProjectionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.discontinuous.DiscontinuousRemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.StanceCMPProjectionMultipliers;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private static final boolean USE_CONTINUOUS_METHOD = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final FinalICPRecursionMultiplier finalICPRecursionMultiplier;
   private final CMPRecursionMultipliers cmpRecursionMultipliers;
   private final StanceCMPProjectionMultipliers stanceCMPProjectionMultipliers;
   private final RemainingStanceCMPProjectionMultipliers remainingStanceCMPProjectionMultipliers;
   private final CurrentStateProjectionMultiplier currentStateProjectionMultiplier;

   private final int maxNumberOfFootstepsToConsider;

   public FootstepRecursionMultiplierCalculator(DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable omega, int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("recursionCalculatorDoubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("recursionCalculatorSingleSupportDuration" + i, registry));
      }

      cmpRecursionMultipliers = new CMPRecursionMultipliers("", maxNumberOfFootstepsToConsider, omega, doubleSupportSplitFraction,
            exitCMPDurationInPercentOfStepTime, registry);
      stanceCMPProjectionMultipliers = new StanceCMPProjectionMultipliers("", omega, doubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime, registry);

      if (USE_CONTINUOUS_METHOD)
      {
         // // FIXME: 8/31/16 
         remainingStanceCMPProjectionMultipliers = new ContinuousRemainingStanceCMPProjectionMultipliers(omega, doubleSupportSplitFraction,
               exitCMPDurationInPercentOfStepTime, registry);
         currentStateProjectionMultiplier = new ContinuousCurrentStateProjectionMultiplier(registry, omega);
      }
      else
      {
         remainingStanceCMPProjectionMultipliers = new DiscontinuousRemainingStanceCMPProjectionMultipliers("", omega, doubleSupportSplitFraction, registry);
         currentStateProjectionMultiplier = new DiscontinuousCurrentStateProjectionMultiplier(registry, omega);
      }

      finalICPRecursionMultiplier = new FinalICPRecursionMultiplier(registry, omega, doubleSupportSplitFraction);

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
      finalICPRecursionMultiplier.reset();
      remainingStanceCMPProjectionMultipliers.reset();
      currentStateProjectionMultiplier.reset();
   }

   public void computeRecursionMultipliers(int numberOfStepsToConsider, boolean isInTransfer, boolean useTwoCMPs)
   {
      reset();

      if (numberOfStepsToConsider > maxNumberOfFootstepsToConsider)
         throw new RuntimeException("Requesting too many steps.");

      finalICPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, isInTransfer);
      stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      cmpRecursionMultipliers.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
   }

   public void computeRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      currentStateProjectionMultiplier.compute(timeRemaining);
      remainingStanceCMPProjectionMultipliers.compute(timeRemaining, doubleSupportDurations, useTwoCMPs, isInTransfer, isInTransferEntry);
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
