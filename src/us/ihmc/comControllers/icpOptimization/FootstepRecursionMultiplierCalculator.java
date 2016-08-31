package us.ihmc.comControllers.icpOptimization;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;

import java.awt.*;
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
   private final DoubleYoVariable oneCMPPreviousStanceProjectionMultiplier;
   private final DoubleYoVariable twoCMPStanceProjectionExitMultiplier;
   private final DoubleYoVariable twoCMPStanceProjectionEntryMultiplier;
   private final DoubleYoVariable twoCMPPreviousStanceProjectionExitMultiplier;

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
      oneCMPPreviousStanceProjectionMultiplier = new DoubleYoVariable("oneCMPPreviousStanceProjectionMultiplier", registry);
      twoCMPStanceProjectionExitMultiplier = new DoubleYoVariable("twoCMPStanceProjectionExitMultiplier", registry);
      twoCMPStanceProjectionEntryMultiplier = new DoubleYoVariable("twoCMPStanceProjectionEntryMultiplier", registry);
      twoCMPPreviousStanceProjectionExitMultiplier = new DoubleYoVariable("twoCMPPreviousStanceProjectionExitMultiplier", registry);

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

         double previousStepDuration = 0.0;
         if (i > 0)
            previousStepDuration = doubleSupportDurations.get(i).getDoubleValue() + singleSupportDurations.get(i).getDoubleValue();

         recursionTime += previousStepDuration;
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
         double previousStepDuration = 0.0;
         if (i > 0)
            previousStepDuration = doubleSupportDurations.get(i).getDoubleValue() + singleSupportDurations.get(i).getDoubleValue();

         recursionTime += previousStepDuration;
         double multiplier = Math.exp(-omega.getDoubleValue() * recursionTime);

         double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * steppingDuration;
         double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;

         double exitRecursion = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         double entryRecursion = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);

         twoCMPRecursionEntryMultipliers.get(i).set(multiplier * entryRecursion);
         twoCMPRecursionExitMultipliers.get(i).set(multiplier * exitRecursion);
      }
   }

   public void computeStanceFootProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      if (useTwoCMPs)
         computeStanceFootTwoCMPProjectionMultipliers(timeRemaining, isInTransfer, isInTransferEntry);
      else
         computeStanceFootOneCMPProjectionMultipliers(timeRemaining, isInTransfer, isInTransferEntry);
   }

   private void computeStanceFootTwoCMPProjectionMultipliers(double timeRemaining, boolean isInTransfer, boolean isInTransferEntry)
   {
      double timeSpentOnInitialDoubleSupport = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double timeSpentOnEndOfCurrentDoubleSupport = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double steppingDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * steppingDuration;
      double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * steppingDuration;

      double entryProjection, exitProjection, previousExitProjection;
      double remainingProjection = Math.exp(omega.getDoubleValue() * timeRemaining);
      if (isInTransfer)
      {
         if (isInTransferEntry)
         {
            entryProjection = Math.exp(-omega.getDoubleValue() * timeSpentOnEndOfCurrentDoubleSupport) -
                  Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - timeSpentOnEndOfCurrentDoubleSupport));
            exitProjection = Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - timeSpentOnEndOfCurrentDoubleSupport)) *
                  (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
            previousExitProjection = remainingProjection - Math.exp(omega.getDoubleValue() * timeSpentOnEndOfCurrentDoubleSupport);
         }
         else
         {
            entryProjection = remainingProjection - Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - timeSpentOnEndOfCurrentDoubleSupport));
            exitProjection = Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - timeSpentOnEndOfCurrentDoubleSupport)) * (1.0 - Math
                  .exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
            previousExitProjection = 0.0;
         }

      }
      else
      {
         entryProjection = 0.0;
         exitProjection = remainingProjection - Math.exp(-omega.getDoubleValue() * timeSpentOnInitialDoubleSupport);
         previousExitProjection = 0.0;
      }
      twoCMPStanceProjectionEntryMultiplier.set(entryProjection);
      twoCMPStanceProjectionExitMultiplier.set(exitProjection);
      twoCMPPreviousStanceProjectionExitMultiplier.set(previousExitProjection);
   }

   private void computeStanceFootOneCMPProjectionMultipliers(double timeRemaining, boolean isInTransfer, boolean isInTransferEntry)
   {
      double timeSpentOnInitialDoubleSupport = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue();
      double timeSpentOnEndOfDoubleSupport = (1 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDuration.getDoubleValue();

      double remainingProjection = Math.exp(omega.getDoubleValue() * timeRemaining);

      double projectionDuration = timeSpentOnInitialDoubleSupport;
      if (isInTransfer)
         projectionDuration += singleSupportDuration.getDoubleValue();

      double totalStanceProjection = remainingProjection - Math.exp(-omega.getDoubleValue() * projectionDuration);
      double previousStanceProjection = 0.0;

      if (isInTransferEntry)
      {
         totalStanceProjection = Math.exp(omega.getDoubleValue() * timeSpentOnEndOfDoubleSupport) - Math.exp(-omega.getDoubleValue() * projectionDuration);
         previousStanceProjection = remainingProjection - Math.exp(omega.getDoubleValue() * timeSpentOnEndOfDoubleSupport);
      }

      oneCMPStanceProjectionMultiplier.set(totalStanceProjection);
      oneCMPPreviousStanceProjectionMultiplier.set(previousStanceProjection);
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

   public double getTwoCMPPreviousStanceProjectionExitMutliplier(boolean useTwoCMPs)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Trying to get one CMP recursion multiplier.");

      return twoCMPPreviousStanceProjectionExitMultiplier.getDoubleValue();
   }

   public double getOneCMPStanceProjectionMultiplier(boolean useTwoCMPs)
   {
      if (useTwoCMPs)
         throw new RuntimeException("Trying to get two CMP recursion multiplier.");

      return oneCMPStanceProjectionMultiplier.getDoubleValue();
   }

   public double getOneCMPPreviousStanceProjectionMultiplier(boolean useTwoCMPs)
   {
      if (useTwoCMPs)
         throw new RuntimeException("Trying to get two CMP recursion multiplier.");

      return oneCMPPreviousStanceProjectionMultiplier.getDoubleValue();
   }

   public void computePredictedICPTouchdownPosition(int numberOfFootstepsToConsider, ArrayList<Footstep> upcomingFootsteps, ArrayList<FrameVector2d> entryOffsets,
                                                    ArrayList<FrameVector2d> exitOffsets, FramePoint2d finalICP, FramePoint2d stanceExitCMP,
                                                    FramePoint2d stanceEntryCMP, boolean useTwoCMPs, boolean isInTransfer, FramePoint2d touchdownICPToPack)
   {
      if (useTwoCMPs)
         computePredictedICPTouchdownPositionTwoCMPs(numberOfFootstepsToConsider, upcomingFootsteps, entryOffsets, exitOffsets, finalICP, stanceExitCMP,
                                                     stanceEntryCMP, isInTransfer, touchdownICPToPack);
      else
         computePredictedICPTouchdownPositionOneCMP(numberOfFootstepsToConsider, upcomingFootsteps, finalICP, stanceExitCMP, isInTransfer, touchdownICPToPack);
   }

   private final FramePoint2d tmpPoint = new FramePoint2d();
   private void computePredictedICPTouchdownPositionOneCMP(int numberOfFootstepsToConsider, ArrayList<Footstep> upcomingFootsteps, FramePoint2d finalICP,
                                                           FramePoint2d stanceCMP, boolean isInTransfer, FramePoint2d touchdownICPToPack)
   {
      touchdownICPToPack.set(finalICP);
      touchdownICPToPack.scale(finalICPRecursionMultiplier.getDoubleValue());

      double initialDoubleSupportTime = doubleSupportDurations.get(1).getDoubleValue() * doubleSupportSplitFraction.getDoubleValue();
      double stanceProjectionTime = initialDoubleSupportTime;
      if (isInTransfer) stanceProjectionTime += singleSupportDurations.get(0).getDoubleValue();
      double stanceCMPProjection = 1.0 - Math.exp(-omega.getDoubleValue() * stanceProjectionTime);

      tmpPoint.set(stanceCMP);
      tmpPoint.scale(stanceCMPProjection);
      touchdownICPToPack.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         upcomingFootsteps.get(i).getPosition2d(tmpPoint);
         tmpPoint.scale(oneCMPRecursionMultipliers.get(i).getDoubleValue());

         touchdownICPToPack.add(tmpPoint);
      }
   }

   private final FramePoint2d tmpEntry = new FramePoint2d();
   private final FramePoint2d tmpExit = new FramePoint2d();
   private void computePredictedICPTouchdownPositionTwoCMPs(int numberOfFootstepsToConsider, ArrayList<Footstep> upcomingFootsteps,
                                                            ArrayList<FrameVector2d> entryOffsets, ArrayList<FrameVector2d> exitOffsets, FramePoint2d finalICP,
                                                            FramePoint2d stanceExitCMP, FramePoint2d stanceEntryCMP, boolean isInTransfer, FramePoint2d touchdownICPToPack)
   {
      touchdownICPToPack.set(finalICP);
      touchdownICPToPack.scale(finalICPRecursionMultiplier.getDoubleValue());

      double initialDoubleSupportTime = doubleSupportDurations.get(1).getDoubleValue() * doubleSupportSplitFraction.getDoubleValue();
      double endOfDoubleSupportTime = doubleSupportDurations.get(0).getDoubleValue() * (1.0 - doubleSupportSplitFraction.getDoubleValue());
      double timeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * (doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue());
      double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * (doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue());

      double stanceExitProjectionTime = initialDoubleSupportTime;
      double stanceExitMultiplier = 1.0;
      double stanceEntryProjectionTime = 0.0;
      double stanceEntryMultiplier = 0.0;
      if (isInTransfer)
      {
         stanceExitMultiplier = Math.exp(-omega.getDoubleValue() * (timeSpentOnEntryCMP - endOfDoubleSupportTime));
         stanceExitProjectionTime = timeSpentOnExitCMP;

         stanceEntryMultiplier = 1.0;
         stanceEntryProjectionTime = timeSpentOnEntryCMP - endOfDoubleSupportTime;
      }
      double stanceEntryCMPProjection = stanceEntryMultiplier * (1.0 - Math.exp(-omega.getDoubleValue() * stanceEntryProjectionTime));
      double stanceExitCMPProjection = stanceExitMultiplier * (1.0 - Math.exp(-omega.getDoubleValue() * stanceExitProjectionTime));

      tmpEntry.set(stanceEntryCMP);
      tmpExit.set(stanceExitCMP);
      tmpEntry.scale(stanceEntryCMPProjection);
      tmpExit.scale(stanceExitCMPProjection);

      touchdownICPToPack.add(tmpEntry);
      touchdownICPToPack.add(tmpExit);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         upcomingFootsteps.get(i).getPosition2d(tmpPoint);

         tmpEntry.set(tmpPoint);
         tmpExit.set(tmpPoint);
         tmpEntry.add(entryOffsets.get(i));
         tmpExit.add(exitOffsets.get(i));

         tmpEntry.scale(twoCMPRecursionEntryMultipliers.get(i).getDoubleValue());
         tmpExit.scale(twoCMPRecursionExitMultipliers.get(i).getDoubleValue());

         tmpPoint.set(tmpEntry);
         tmpPoint.add(tmpExit);

         touchdownICPToPack.add(tmpPoint);
      }

   }
}
