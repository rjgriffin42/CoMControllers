package us.ihmc.comControllers.icpOptimization;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> oneCMPRecursionMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> twoCMPRecursionEntryMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> twoCMPRecursionExitMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final DoubleYoVariable oneCMPStanceProjectionMultiplier;
   private final DoubleYoVariable twoCMPStanceProjectionExitMultiplier;
   private final DoubleYoVariable twoCMPStanceProjectionEntryMultiplier;

   private final DoubleYoVariable doubleSupportDuration;
   private final DoubleYoVariable singleSupportDuration;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;
   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable omega;
   private final DoubleYoVariable finalICPRecursionMultiplier;

   private final int maxNumberOfFootstepsToConsider;

   public FootstepRecursionMultiplierCalculator(DoubleYoVariable doubleSupportDuration, DoubleYoVariable singleSupportDuration,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable doubleSupportSplitFraction, DoubleYoVariable omega,
         int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.doubleSupportDuration = doubleSupportDuration;
      this.singleSupportDuration = singleSupportDuration;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;
      this.omega = omega;
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         oneCMPRecursionMultipliers.add(new DoubleYoVariable("oneCMPRecursionMultiplier" + i, registry));
         twoCMPRecursionEntryMultipliers.add(new DoubleYoVariable("twoCMPRecursionEntryMultiplier" + i, registry));
         twoCMPRecursionExitMultipliers.add(new DoubleYoVariable("twoCMPRecursionExitMultiplier" + i, registry));

         doubleSupportDurations.add(new DoubleYoVariable("recursionCalculatorDoubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("recursionCalculatorSingleSupportDuration" + i, registry));
      }

      oneCMPStanceProjectionMultiplier = new DoubleYoVariable("oneCMPStanceProjectionMultiplier", registry);
      twoCMPStanceProjectionExitMultiplier = new DoubleYoVariable("twoCMPStanceProjectionExitMultiplier", registry);
      twoCMPStanceProjectionEntryMultiplier = new DoubleYoVariable("twoCMPStanceProjectionEntryMultiplier", registry);

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
         oneCMPRecursionMultipliers.get(i).set(0.0);
         twoCMPRecursionEntryMultipliers.get(i).set(0.0);
         twoCMPRecursionExitMultipliers.get(i).set(0.0);
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

         recursionTime += steppingDuration;
         oneCMPRecursionMultipliers.get(i).set(Math.exp(-omega.getDoubleValue() * recursionTime) * stepRecursion);
      }

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
         timeToFinish = -timeSpentOnEndDoubleSupportCurrent + firstStepTime;
      else
         timeToFinish = timeSpentOnInitialDoubleSupportUpcoming;

      double recursionTime = timeToFinish;
      for (int i = 0; i < numberOfStepsToConsider; i++)
      {
         double steppingDuration = singleSupportDurations.get(i + 1).getDoubleValue() + doubleSupportDurations.get(i + 1).getDoubleValue();
         recursionTime += steppingDuration;
         double multiplier = Math.exp(-omega.getDoubleValue() * recursionTime);

         double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * steppingDuration;
         double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;

         double exitRecursion = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         double entryRecursion = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);

         twoCMPRecursionEntryMultipliers.get(i).set(multiplier * entryRecursion);
         twoCMPRecursionExitMultipliers.get(i).set(multiplier * exitRecursion);
      }
   }

   public void computeStanceFootProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer)
   {
      if (useTwoCMPs)
         computeStanceFootTwoCMPProjectionMultipliers(timeRemaining, isInTransfer);
      else
         computeStanceFootOneCMPProjectionMultipliers(timeRemaining, isInTransfer);
   }

   private void computeStanceFootTwoCMPProjectionMultipliers(double timeRemaining, boolean isInTransfer)
   {
      double timeSpentOnInitialDoubleSupport = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue();
      double timeSpentOnEndDoubleSupport = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDuration.getDoubleValue();

      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;

      double entryProjection, exitProjection;
      double remainingProjection = Math.exp(omega.getDoubleValue() * timeRemaining);
      if (isInTransfer)
      {
         entryProjection = remainingProjection - Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - timeSpentOnEndDoubleSupport));
         exitProjection = Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - timeSpentOnEndDoubleSupport)) *
               (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP));
      }
      else
      {
         entryProjection = 0.0;
         exitProjection = remainingProjection - Math.exp(-omega.getDoubleValue() * timeSpentOnInitialDoubleSupport);
      }
      twoCMPStanceProjectionEntryMultiplier.set(entryProjection);
      twoCMPStanceProjectionExitMultiplier.set(exitProjection);
   }

   private void computeStanceFootOneCMPProjectionMultipliers(double timeRemaining, boolean isInTransfer)
   {
      double timeSpentOnInitialDoubleSupport = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue();

      double remainingProjection = Math.exp(omega.getDoubleValue() * timeRemaining);

      double projectionDuration = timeSpentOnInitialDoubleSupport;
      if (isInTransfer)
         projectionDuration += singleSupportDuration.getDoubleValue();

      oneCMPStanceProjectionMultiplier.set(remainingProjection - Math.exp(-omega.getDoubleValue() * projectionDuration));
   }

   private void computeFinalICPRecursionMultiplier(double totalTime, double omega)
   {
      finalICPRecursionMultiplier.set(Math.exp(-omega * totalTime));
   }

   public double getOneCMPRecursionMultiplier(int footstepIndex, boolean useTwoCMPs)
   {
      if (useTwoCMPs)
         throw new RuntimeException("Trying to get two CMP recursion multiplier.");

      return oneCMPRecursionMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getTwoCMPRecursionEntryMultiplier(int footstepIndex, boolean useTwoCMPs)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPRecursionEntryMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getTwoCMPRecursionExitMultiplier(int footstepIndex, boolean useTwoCMPs)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPRecursionExitMultipliers.get(footstepIndex).getDoubleValue();
   }

   public double getFinalICPRecursionMultiplier()
   {
      return finalICPRecursionMultiplier.getDoubleValue();
   }

   public double getTwoCMPStanceProjectionEntryMutliplier(boolean useTwoCMPs)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPStanceProjectionEntryMultiplier.getDoubleValue();
   }

   public double getTwoCMPStanceProjectionExitMutliplier(boolean useTwoCMPs)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPStanceProjectionExitMultiplier.getDoubleValue();
   }

   public double getOneCMPStanceProjectionMultiplier(boolean useTwoCMPs)
   {
      if (useTwoCMPs)
         throw new RuntimeException("Trying to get two CMP recursion multiplier.");

      return oneCMPStanceProjectionMultiplier.getDoubleValue();
   }
}
