package us.ihmc.comControllers.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.*;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.ContinuousCurrentStateProjectionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.ContinuousRemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.StanceCMPProjectionMultipliers;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;

import java.util.ArrayList;

public class FootstepRecursionMultiplierCalculator
{
   private static final boolean USE_CONTINUOUS_METHOD = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();

   private final FinalICPRecursionMultiplier finalICPRecursionMultiplier;
   private final CMPRecursionMultipliers cmpRecursionMultipliers;
   private final StanceCMPProjectionMultipliers stanceCMPProjectionMultipliers;
   private final ContinuousRemainingStanceCMPProjectionMultipliers remainingStanceCMPProjectionMultipliers;
   private final ContinuousCurrentStateProjectionMultiplier currentStateProjectionMultiplier;

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
         remainingStanceCMPProjectionMultipliers = new ContinuousRemainingStanceCMPProjectionMultipliers(omega, doubleSupportSplitFraction,
               exitCMPDurationInPercentOfStepTime, registry);
         currentStateProjectionMultiplier = new ContinuousCurrentStateProjectionMultiplier(registry, omega, doubleSupportSplitFraction);
      }
      /*
      else
      {
         remainingStanceCMPProjectionMultipliers = new DiscontinuousRemainingStanceCMPProjectionMultipliers("", omega, doubleSupportSplitFraction, registry);
         currentStateProjectionMultiplier = new DiscontinuousCurrentStateProjectionMultiplier(registry, omega);
      }
      */

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
      currentStateProjectionMultiplier.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      remainingStanceCMPProjectionMultipliers.compute(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer, isInTransferEntry);
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

   public void computeNominalICPPoints(FramePoint2d finalICP, ArrayList<Footstep> footsteps, ArrayList<FrameVector2d> entryOffsets,
                                       ArrayList<FrameVector2d> exitOffsets, FramePoint2d previousExitCMP, FramePoint2d entryCMP, FramePoint2d exitCMP,
                                       int numberOfFootstepsToConsider, FramePoint2d nominalPredictedEndOfStateICP, FramePoint2d nominalBeginningOfStateICPToPack,
                                       FramePoint2d nominalReferenceICPToPack)
   {
      nominalPredictedEndOfStateICP.set(finalICP);
      nominalPredictedEndOfStateICP.scale(getFinalICPRecursionMultiplier());

      tmpPoint.set(entryCMP);
      tmpPoint.scale(getStanceEntryCMPProjectionMultiplier());

      nominalPredictedEndOfStateICP.add(tmpPoint);

      tmpPoint.set(exitCMP);
      tmpPoint.scale(getStanceExitCMPProjectionMultiplier());

      nominalPredictedEndOfStateICP.add(tmpPoint);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         footsteps.get(i).getPosition2d(tmpPoint);
         tmpEntry.set(tmpPoint);
         tmpExit.set(tmpPoint);

         tmpEntry.add(entryOffsets.get(i));
         tmpExit.add(exitOffsets.get(i));

         tmpEntry.scale(getCMPRecursionEntryMultiplier(i));
         tmpExit.scale(getCMPRecursionExitMultiplier(i));

         nominalPredictedEndOfStateICP.add(tmpEntry);
         nominalPredictedEndOfStateICP.add(tmpExit);
      }

      endOfStateICPMatrix.set(0, 0, nominalPredictedEndOfStateICP.getX());
      endOfStateICPMatrix.set(0, 1, nominalPredictedEndOfStateICP.getY());

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

      nominalBeginningOfStateICPToPack.setX(boundaryConditionMatrix.get(0, 0));
      nominalBeginningOfStateICPToPack.setY(boundaryConditionMatrix.get(0, 1));

      CommonOps.mult(remainingStanceCMPProjectionMultipliers.getCubicProjectionMatrix(), boundaryConditionMatrix, referenceICPMatrix);

      nominalReferenceICPToPack.setX(referenceICPMatrix.get(0, 0));
      nominalReferenceICPToPack.setY(referenceICPMatrix.get(0, 1));
   }
}
