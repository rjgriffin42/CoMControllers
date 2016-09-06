package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.RemainingStanceCMPProjectionMultipliers;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation.CubicProjectionDerivativeMatrix;
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
   private final CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix;

   private final ExitCMPProjectionMatrix exitCMPProjectionMatrix;
   private final EntryCMPProjectionMatrix entryCMPProjectionMatrix;
   private final PreviousExitCMPProjectionMatrix previousExitCMPProjectionMatrix;

   private final DenseMatrix64F exitMultiplierMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F entryMultiplierMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F previousExitMultiplierMatrix = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable exitMultiplier = new DoubleYoVariable("remainingStanceExitCMPProjectionMultiplier", registry);
   private final DoubleYoVariable entryMultiplier = new DoubleYoVariable("remainingStanceEntryCMPProjectionMultiplier", registry);
   private final DoubleYoVariable previousExitMultiplier = new DoubleYoVariable("remainingStancePreviousExitCMPProjectionMultiplier", registry);

   private final DoubleYoVariable omega;

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;
   private final DoubleYoVariable doubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   public ContinuousRemainingStanceCMPProjectionMultipliers(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime,
                                                            YoVariableRegistry parentRegistry)
   {
      this.omega = omega;

      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;
      this.exitCMPProjectionMatrix = new ExitCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
      this.entryCMPProjectionMatrix = new EntryCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, startOfSplineTime);
      this.previousExitCMPProjectionMatrix = new PreviousExitCMPProjectionMatrix(omega, doubleSupportSplitRatio);

      cubicProjectionMatrix = new CubicProjectionMatrix();
      cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

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
      {
         computeUsingSpline(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      }
      else
      {
         double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;
         if (timeInState < startOfSplineTime.getDoubleValue())
         {
            computeInSingleSupportFirstSegment(timeRemaining, doubleSupportDurations, singleSupportDurations);
         }
         else if (timeInState >= endOfSplineTime.getDoubleValue())
         {
            computeInSingleSupportThirdSegment(timeRemaining);
         }
         else
         {
            computeUsingSpline(timeRemaining, doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
         }
      }
   }

   public void computeUsingSpline(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer)
   {
      if (isInTransfer)
      {
         cubicProjectionMatrix.setSegmentDuration(doubleSupportDurations.get(0).getDoubleValue());
         cubicProjectionDerivativeMatrix.setSegmentDuration(doubleSupportDurations.get(0).getDoubleValue());
      }
      else
      {
         cubicProjectionMatrix.setSegmentDuration(totalTrajectoryTime.getDoubleValue());
         cubicProjectionDerivativeMatrix.setSegmentDuration(totalTrajectoryTime.getDoubleValue());
      }

      cubicProjectionMatrix.update(timeRemaining);
      cubicProjectionDerivativeMatrix.update(timeRemaining);

      exitCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      entryCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
      previousExitCMPProjectionMatrix.compute(doubleSupportDurations, isInTransfer);

      CommonOps.mult(cubicProjectionMatrix, exitCMPProjectionMatrix, exitMultiplierMatrix);
      CommonOps.mult(cubicProjectionMatrix, entryCMPProjectionMatrix, entryMultiplierMatrix);
      CommonOps.mult(cubicProjectionMatrix, previousExitCMPProjectionMatrix, previousExitMultiplierMatrix);

      exitMultiplier.set(exitMultiplierMatrix.get(0, 0));
      entryMultiplier.set(entryMultiplierMatrix.get(0, 0));
      previousExitMultiplier.set(previousExitMultiplierMatrix.get(0, 0));

      if (!useTwoCMPs && !isInTransfer)
         exitMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeRemaining));
   }

   private void computeInSingleSupportFirstSegment(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations,
                                                   ArrayList<DoubleYoVariable> singleSupportDurations)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;
      double stepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

      double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double timeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * stepDuration;
      double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * stepDuration;

      previousExitMultiplier.set(0.0);

      double currentTimeRecursion = Math.exp(omega.getDoubleValue() * (timeInState + endOfDoubleSupportDuration - timeSpentOnEntryCMP));
      entryMultiplier.set(1.0 - currentTimeRecursion);

      double totalTimeOnExit = upcomingInitialDoubleSupportDuration - timeSpentOnExitCMP;
      exitMultiplier.set(currentTimeRecursion * (1.0 - Math.exp(omega.getDoubleValue() * totalTimeOnExit)));
   }

   private void computeInSingleSupportThirdSegment(double timeRemaining)
   {
      previousExitMultiplier.set(0.0);
      entryMultiplier.set(0.0);
      exitMultiplier.set(Math.exp(-omega.getDoubleValue() * timeRemaining));
   }

   public double getRemainingExitMultiplier()
   {
      return exitMultiplier.getDoubleValue();
   }

   public double getRemainingEntryMultiplier()
   {
      return entryMultiplier.getDoubleValue();
   }

   public double getRemainingPreviousExitMultiplier()
   {
      return previousExitMultiplier.getDoubleValue();
   }

   public DenseMatrix64F getExitMultiplierMatrix()
   {
      return exitCMPProjectionMatrix;
   }

   public DenseMatrix64F getEntryMultiplierMatrix()
   {
      return entryCMPProjectionMatrix;
   }

   public DenseMatrix64F getPreviousExitMultiplierMatrix()
   {
      return previousExitCMPProjectionMatrix;
   }

   public DenseMatrix64F getCubicProjectionMatrix()
   {
      return cubicProjectionMatrix;
   }

   public DenseMatrix64F getCubicProjectionDerivativeMatrix()
   {
      return cubicProjectionDerivativeMatrix;
   }
}
