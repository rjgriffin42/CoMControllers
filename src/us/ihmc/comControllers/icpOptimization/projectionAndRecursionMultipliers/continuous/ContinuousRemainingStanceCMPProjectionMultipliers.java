package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.RemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation.CubicProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.EntryCMPProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.ExitCMPProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.PreviousExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;
import java.util.Map;

public class ContinuousRemainingStanceCMPProjectionMultipliers implements RemainingStanceCMPProjectionMultipliers
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CubicProjectionMatrix cubicProjectionMatrix;

   private final ExitCMPProjectionMatrix exitCMPProjectionMatrix;
   private final EntryCMPProjectionMatrix entryCMPProjectionMatrix;
   private final PreviousExitCMPProjectionMatrix previousExitCMPProjectionMatrix;

   private final DenseMatrix64F exitMultiplierMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F entryMultiplierMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F previousExitMultiplierMatrix = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable exitMultiplier = new DoubleYoVariable("remainingStanceExitCMPProjectionMultiplier", registry);
   private final DoubleYoVariable entryMultiplier = new DoubleYoVariable("remainingStanceEntryCMPProjectionMultiplier", registry);
   private final DoubleYoVariable previousExitMultiplier = new DoubleYoVariable("remainingStancePreviousExitCMPProjectionMultiplier", registry);

   public ContinuousRemainingStanceCMPProjectionMultipliers(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, YoVariableRegistry parentRegistry)
   {
      this.exitCMPProjectionMatrix = new ExitCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime);
      this.entryCMPProjectionMatrix = new EntryCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime);
      this.previousExitCMPProjectionMatrix = new PreviousExitCMPProjectionMatrix(omega, doubleSupportSplitRatio);

      cubicProjectionMatrix = new CubicProjectionMatrix();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitCMPProjectionMatrix.reset();
      entryCMPProjectionMatrix.reset();
      previousExitCMPProjectionMatrix.reset();

      exitMultiplier.set(0.0);
      entryMultiplier.set(0.0);
      previousExitMultiplier.set(0.0);

      exitMultiplierMatrix.zero();
      entryMultiplierMatrix.zero();
      previousExitMultiplierMatrix.zero();
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      if (isInTransfer)
         cubicProjectionMatrix.setSegmentDuration(doubleSupportDurations.get(0).getDoubleValue());
      else
         cubicProjectionMatrix.setSegmentDuration(singleSupportDurations.get(0).getDoubleValue());

      cubicProjectionMatrix.update(timeRemaining);

      exitCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      entryCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      previousExitCMPProjectionMatrix.compute(doubleSupportDurations, isInTransfer);

      CommonOps.mult(cubicProjectionMatrix, exitCMPProjectionMatrix, exitMultiplierMatrix);
      CommonOps.mult(cubicProjectionMatrix, entryCMPProjectionMatrix, entryMultiplierMatrix);
      CommonOps.mult(cubicProjectionMatrix, previousExitCMPProjectionMatrix, previousExitMultiplierMatrix);

      exitMultiplier.set(exitMultiplierMatrix.get(0, 0));
      entryMultiplier.set(entryMultiplierMatrix.get(0, 0));
      previousExitMultiplier.set(previousExitMultiplierMatrix.get(0, 0));
   }

   public double getExitMultiplier()
   {
      return exitMultiplier.getDoubleValue();
   }

   public double getEntryMultiplier()
   {
      return entryMultiplier.getDoubleValue();
   }

   public double getPreviousExitMultiplier()
   {
      return previousExitMultiplier.getDoubleValue();
   }
}
