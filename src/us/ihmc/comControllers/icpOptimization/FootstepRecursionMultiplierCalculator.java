package us.ihmc.comControllers.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.*;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.ContinuousCurrentStateProjectionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.ContinuousRemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.StanceCMPProjectionMultipliers;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private static final boolean USE_CONTINUOUS_METHOD = true;

   private static final String namePrefix = "controller";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final FinalICPRecursionMultiplier finalICPRecursionMultiplier;
   private final CMPRecursionMultipliers cmpRecursionMultipliers;
   private final StanceCMPProjectionMultipliers stanceCMPProjectionMultipliers;
   private final ContinuousRemainingStanceCMPProjectionMultipliers remainingStanceCMPProjectionMultipliers;
   private final ContinuousCurrentStateProjectionMultiplier currentStateProjectionMultiplier;

   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final DoubleYoVariable maximumSplineDuration;
   private final DoubleYoVariable minimumSplineDuration;
   private final DoubleYoVariable minimumTimeToSpendOnExitCMP;
   private final DoubleYoVariable totalTrajectoryTime;
   private final DoubleYoVariable timeSpentOnInitialCMP;
   private final DoubleYoVariable timeSpentOnFinalCMP;
   private final DoubleYoVariable progressionInPercent;
   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;

   private final int maxNumberOfFootstepsToConsider;

   public FootstepRecursionMultiplierCalculator(CapturePointPlannerParameters icpPlannerParameters, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
                                                DoubleYoVariable doubleSupportSplitFraction, DoubleYoVariable omega, int maxNumberOfFootstepsToConsider,
                                                YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         doubleSupportDurations.add(new DoubleYoVariable("recursionCalculatorDoubleSupportDuration" + i, registry));
         singleSupportDurations.add(new DoubleYoVariable("recursionCalculatorSingleSupportDuration" + i, registry));
      }

      cmpRecursionMultipliers = new CMPRecursionMultipliers("", maxNumberOfFootstepsToConsider, omega, doubleSupportSplitFraction,
            exitCMPDurationInPercentOfStepTime, registry);
      stanceCMPProjectionMultipliers = new StanceCMPProjectionMultipliers("", omega, doubleSupportSplitFraction, exitCMPDurationInPercentOfStepTime, registry);

      maximumSplineDuration = new DoubleYoVariable(namePrefix + "MaximumSplineDuration", registry);
      minimumSplineDuration = new DoubleYoVariable(namePrefix + "MinimumSplineDuration", registry);
      minimumTimeToSpendOnExitCMP = new DoubleYoVariable(namePrefix + "MinimumTimeToSpendOnExitCMP", registry);

      minimumSplineDuration.set(0.1);
      maximumSplineDuration.set(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCMPSwitch());
      minimumTimeToSpendOnExitCMP.set(icpPlannerParameters.getMinTimeToSpendOnExitCMPInSingleSupport());

      totalTrajectoryTime = new DoubleYoVariable(namePrefix + "TotalTrajectoryTime", registry);
      timeSpentOnInitialCMP = new DoubleYoVariable(namePrefix + "TimeSpentOnInitialCMP", registry);
      timeSpentOnFinalCMP = new DoubleYoVariable(namePrefix + "TimeSpentOnFinalCMP", registry);
      progressionInPercent = new DoubleYoVariable(namePrefix + "ProgressionInPercent", registry);
      startOfSplineTime = new DoubleYoVariable(namePrefix + "StartOfSplineTime", registry);
      endOfSplineTime = new DoubleYoVariable(namePrefix + "EndOfSplineTime", registry);

      remainingStanceCMPProjectionMultipliers = new ContinuousRemainingStanceCMPProjectionMultipliers(omega, doubleSupportSplitFraction,
               exitCMPDurationInPercentOfStepTime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime, registry);
      currentStateProjectionMultiplier = new ContinuousCurrentStateProjectionMultiplier(registry, omega, doubleSupportSplitFraction,
                                                                                        exitCMPDurationInPercentOfStepTime, startOfSplineTime, endOfSplineTime,
                                                                                        totalTrajectoryTime);

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

      finalICPRecursionMultiplier.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      stanceCMPProjectionMultipliers.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      cmpRecursionMultipliers.compute(numberOfStepsToConsider, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
   }

   public void computeRemainingProjectionMultipliers(double timeRemaining, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {

      if (useTwoCMPs)
      {
         updateSegmentedSingleSupportTrajectory(isInTransfer);
      }

      currentStateProjectionMultiplier.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      remainingStanceCMPProjectionMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, isInTransferEntry);
   }

   private void updateSegmentedSingleSupportTrajectory(boolean isInTransfer)
   {
      if (isInTransfer)
      {
         double doubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
         double steppingDuration = singleSupportDurations.get(0).getDoubleValue() + doubleSupportDuration;

         double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
         double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

         double doubleSupportTimeSpentBeforeEntryCornerPoint = doubleSupportDuration * doubleSupportSplitFraction.getDoubleValue();
         double doubleSupportTimeSpentAfterEntryCornerPoint = doubleSupportDuration * (1.0 - doubleSupportSplitFraction.getDoubleValue());

         double timeRemainingOnEntryCMP = totalTimeSpentOnEntryCMP - doubleSupportTimeSpentBeforeEntryCornerPoint;
         double timeToSpendOnFinalCMPBeforeDoubleSupport = totalTimeSpentOnExitCMP - doubleSupportTimeSpentAfterEntryCornerPoint;

         timeSpentOnInitialCMP.set(timeRemainingOnEntryCMP);
         timeSpentOnFinalCMP.set(timeToSpendOnFinalCMPBeforeDoubleSupport);
         totalTrajectoryTime.set(timeRemainingOnEntryCMP + timeToSpendOnFinalCMPBeforeDoubleSupport);

         double alpha = 0.50;
         double minTimeOnExitCMP = minimumTimeToSpendOnExitCMP.getDoubleValue();
         minTimeOnExitCMP = Math.min(minTimeOnExitCMP, timeSpentOnFinalCMP.getDoubleValue() - alpha * minimumSplineDuration.getDoubleValue());

         double startOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() - alpha * maximumSplineDuration.getDoubleValue();
         startOfSplineTime = Math.max(startOfSplineTime, 0.0);
         this.startOfSplineTime.set(startOfSplineTime);

         double endOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() + (1.0 - alpha) * maximumSplineDuration.getDoubleValue();
         endOfSplineTime = Math.min(endOfSplineTime, totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP);
         if (endOfSplineTime > totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP)
         {
            endOfSplineTime = totalTrajectoryTime.getDoubleValue() - minTimeOnExitCMP;
            startOfSplineTime = timeSpentOnInitialCMP.getDoubleValue() - (endOfSplineTime - timeSpentOnInitialCMP.getDoubleValue());
         }
         this.startOfSplineTime.set(startOfSplineTime);
         this.endOfSplineTime.set(endOfSplineTime);
      }
      else
      {
         timeSpentOnInitialCMP.set(Double.NaN);
         timeSpentOnFinalCMP.set(Double.NaN);
         totalTrajectoryTime.set(Double.NaN);
         startOfSplineTime.set(Double.NaN);
         endOfSplineTime.set(Double.NaN);
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
      return stanceCMPProjectionMultipliers.getExitMultiplier();
   }

   public double getStanceEntryCMPProjectionMultiplier()
   {
      return stanceCMPProjectionMultipliers.getEntryMultiplier();
   }

   public double getRemainingStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getRemainingExitMultiplier();
   }

   public double getRemainingStanceEntryCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getRemainingEntryMultiplier();
   }

   public double getRemainingPreviousStanceExitCMPProjectionMultiplier()
   {
      return remainingStanceCMPProjectionMultipliers.getRemainingPreviousExitMultiplier();
   }

   public double getCurrentStateProjectionMultiplier()
   {
      return currentStateProjectionMultiplier.getDoubleValue();
   }

   private final FramePoint2d tmpPoint = new FramePoint2d();
   private final FramePoint2d tmpEntry = new FramePoint2d();
   private final FramePoint2d tmpExit = new FramePoint2d();

   private final DenseMatrix64F entryCMPMatrix = new DenseMatrix64F(1, 2);
   private final DenseMatrix64F exitCMPMatrix = new DenseMatrix64F(1, 2);
   private final DenseMatrix64F previousExitCMPMatrix = new DenseMatrix64F(1, 2);
   private final DenseMatrix64F endOfStateICPMatrix = new DenseMatrix64F(1, 2);

   private final DenseMatrix64F boundaryConditionMatrix = new DenseMatrix64F(4, 2);
   private final DenseMatrix64F referenceICPMatrix = new DenseMatrix64F(1, 2);
   private final DenseMatrix64F referenceICPVelocityMatrix = new DenseMatrix64F(1, 2);

   public void computeICPPoints(FramePoint2d finalICP, ArrayList<YoFramePoint2d> footstepLocations, ArrayList<FrameVector2d> entryOffsets,
                                       ArrayList<FrameVector2d> exitOffsets, FramePoint2d previousExitCMP, FramePoint2d entryCMP, FramePoint2d exitCMP,
                                       int numberOfFootstepsToConsider, FramePoint2d predictedEndOfStateICP, FramePoint2d beginningOfStateICPToPack,
                                       FramePoint2d referenceICPToPack, FrameVector2d referenceICPVelocityToPack)
   {
      predictedEndOfStateICP.set(finalICP);
      predictedEndOfStateICP.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPProjectionMultiplier());

      predictedEndOfStateICP.add(tmpPoint);

      tmpPoint.set(exitCMP);
      tmpPoint.scale(getStanceExitCMPProjectionMultiplier());

      predictedEndOfStateICP.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpEntry.set(footstepLocations.get(i).getFrameTuple2d());
         tmpExit.set(footstepLocations.get(i).getFrameTuple2d());

         tmpEntry.add(entryOffsets.get(i));
         tmpExit.add(exitOffsets.get(i));

         tmpEntry.scale(getCMPRecursionEntryMultiplier(i));
         tmpExit.scale(getCMPRecursionExitMultiplier(i));

         predictedEndOfStateICP.add(tmpEntry);
         predictedEndOfStateICP.add(tmpExit);
      }

      endOfStateICPMatrix.set(0, 0, predictedEndOfStateICP.getX());
      endOfStateICPMatrix.set(0, 1, predictedEndOfStateICP.getY());

      entryCMPMatrix.set(0, 0, entryCMP.getX());
      entryCMPMatrix.set(0, 1, entryCMP.getY());

      exitCMPMatrix.set(0, 0, exitCMP.getX());
      exitCMPMatrix.set(0, 1, exitCMP.getY());

      if (!previousExitCMP.containsNaN())
      {
         previousExitCMPMatrix.set(0, 0, previousExitCMP.getX());
         previousExitCMPMatrix.set(0, 1, previousExitCMP.getY());
      }
      else
      {
         previousExitCMPMatrix.zero();
      }

      CommonOps.mult(currentStateProjectionMultiplier.getStateEndRecursionMatrix(), endOfStateICPMatrix, boundaryConditionMatrix);
      CommonOps.multAdd(remainingStanceCMPProjectionMultipliers.getEntryMultiplierMatrix(), entryCMPMatrix, boundaryConditionMatrix);
      CommonOps.multAdd(remainingStanceCMPProjectionMultipliers.getExitMultiplierMatrix(), exitCMPMatrix, boundaryConditionMatrix);
      CommonOps.multAdd(remainingStanceCMPProjectionMultipliers.getPreviousExitMultiplierMatrix(), previousExitCMPMatrix, boundaryConditionMatrix);

      beginningOfStateICPToPack.setX(boundaryConditionMatrix.get(0, 0));
      beginningOfStateICPToPack.setY(boundaryConditionMatrix.get(0, 1));

      CommonOps.mult(remainingStanceCMPProjectionMultipliers.getCubicProjectionMatrix(), boundaryConditionMatrix, referenceICPMatrix);
      CommonOps.mult(remainingStanceCMPProjectionMultipliers.getCubicProjectionDerivativeMatrix(), boundaryConditionMatrix, referenceICPVelocityMatrix);

      referenceICPToPack.setX(referenceICPMatrix.get(0, 0));
      referenceICPToPack.setY(referenceICPMatrix.get(0, 1));

      referenceICPVelocityToPack.setX(referenceICPVelocityMatrix.get(0, 0));
      referenceICPVelocityToPack.setY(referenceICPVelocityMatrix.get(0, 1));
   }
}
