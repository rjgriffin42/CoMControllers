package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CurrentStateProjectionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation.CubicProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.StateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class ContinuousCurrentStateProjectionMultiplier extends CurrentStateProjectionMultiplier
{
   private final StateEndRecursionMatrix stateEndRecursionMatrix;
   private final CubicProjectionMatrix cubicProjectionMatrix;

   private final DoubleYoVariable omega;

   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   public ContinuousCurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio,
                                                     DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime,
                                                     DoubleYoVariable totalTrajectoryTime)
   {
      super(registry);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPRatio = exitCMPRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      this.stateEndRecursionMatrix = new StateEndRecursionMatrix(omega, doubleSupportSplitRatio, exitCMPRatio);
      this.cubicProjectionMatrix = new CubicProjectionMatrix();
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                       boolean useTwoCMPs, boolean isInTransfer)
   {
      double projection;
      if (isInTransfer)
      {
         projection = computeInTransfer(doubleSupportDurations, timeRemaining);
      }
      else if (useTwoCMPs)
      {
         projection = computeSegmentedProjection(doubleSupportDurations, singleSupportDurations, timeRemaining);
      }
      else
      {
         projection = Math.exp(omega.getDoubleValue() * timeRemaining);
      }
      this.set(projection);
   }

   public double computeSegmentedProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                                            double timeRemaining)
   {
      double currentTimeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (currentTimeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentProjection(doubleSupportDurations, singleSupportDurations, currentTimeInState);
      else if (currentTimeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentProjection(timeRemaining);
      else
         return computeSecondSegmentProjection(doubleSupportDurations, singleSupportDurations, currentTimeInState);
   }

   public double computeFirstSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                                               double currentTimeInState)
   {
      double steppingDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

      double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double currentEndOfDoubleSupport = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double projectionDuration = currentTimeInState + currentEndOfDoubleSupport + upcomingInitialDoubleSupportDuration - steppingDuration;

      return Math.exp(-omega.getDoubleValue() * projectionDuration);
   }

   public double computeSecondSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double currentTimeInState)
   {
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();
      double timeInSegment = currentTimeInState - startOfSplineTime.getDoubleValue();
      double timeRemainingInSegment = splineDuration - timeInSegment;

      stateEndRecursionMatrix.computeInSingleSupport(doubleSupportDurations, singleSupportDurations, startOfSplineTime.getDoubleValue(), endOfSplineTime.getDoubleValue());

      cubicProjectionMatrix.setSegmentDuration(splineDuration);
      cubicProjectionMatrix.update(timeRemainingInSegment);
      CommonOps.mult(cubicProjectionMatrix, stateEndRecursionMatrix, matrixOut);

      return 1.0 / matrixOut.get(0, 0);
   }

   public double computeThirdSegmentProjection(double timeRemainingInState)
   {
      return Math.exp(-omega.getDoubleValue() * timeRemainingInState);
   }

   public double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining)
   {
      stateEndRecursionMatrix.computeInTransfer(doubleSupportDurations);

      cubicProjectionMatrix.setSegmentDuration(doubleSupportDurations.get(0).getDoubleValue());
      cubicProjectionMatrix.update(timeRemaining);
      CommonOps.mult(cubicProjectionMatrix, stateEndRecursionMatrix, matrixOut);

      return 1.0 / matrixOut.get(0, 0);
   }

   public void reset()
   {
      this.set(0.0);
   }

   public DenseMatrix64F getStateEndRecursionMatrix()
   {
      return stateEndRecursionMatrix;
   }
}
