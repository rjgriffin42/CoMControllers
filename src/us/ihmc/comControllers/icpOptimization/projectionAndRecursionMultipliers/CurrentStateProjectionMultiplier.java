package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicProjectionDerivativeMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingStateEndRecursionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferStateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class CurrentStateProjectionMultiplier
{
   private final CubicProjectionMatrix cubicProjectionMatrix;
   private final CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix;

   private final TransferStateEndRecursionMatrix transferStateEndRecursionMatrix;
   private final SwingStateEndRecursionMatrix swingStateEndRecursionMatrix;

   private final DoubleYoVariable omega;

   private final DoubleYoVariable doubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public CurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      positionMultiplier = new DoubleYoVariable("CurrentStateProjectionMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("CurrentStateVelocityProjectionMultiplier", registry);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      transferStateEndRecursionMatrix = new TransferStateEndRecursionMatrix(omega);
      swingStateEndRecursionMatrix = new SwingStateEndRecursionMatrix(omega, doubleSupportSplitRatio);

      cubicProjectionMatrix = new CubicProjectionMatrix();
      cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();
   }

   public void reset()
   {
      positionMultiplier.set(0.0);
      velocityMultiplier.set(0.0);
   }

   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double timeRemaining,
         boolean useTwoCMPs, boolean isInTransfer)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
      {
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeRemaining);
         velocityMultiplier = computeInTransferVelocity();
      }
      else
      {
         if (useTwoCMPs)
         {
            positionMultiplier = computeSegmentedProjection(doubleSupportDurations, singleSupportDurations, timeRemaining);
            velocityMultiplier = computeSegmentedVelocityProjection(timeRemaining);
         }
         else
         {
            positionMultiplier = computeInSwingOneCMP(timeRemaining);
            velocityMultiplier = computeInSwingOneCMPVelocity();
         }
      }

      this.positionMultiplier.set(positionMultiplier);
      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining)
   {
      transferStateEndRecursionMatrix.compute(doubleSupportDurations);

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicProjectionDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicProjectionDerivativeMatrix.update(timeRemaining);
      cubicProjectionMatrix.setSegmentDuration(splineDuration);
      cubicProjectionMatrix.update(timeRemaining);
      CommonOps.mult(cubicProjectionMatrix, transferStateEndRecursionMatrix, matrixOut);

      return 1.0 / matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMP(double timeRemaining)
   {
      return Math.exp(omega.getDoubleValue() * timeRemaining);
   }

   private double computeSegmentedProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentProjection(doubleSupportDurations, singleSupportDurations, timeInState);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentProjection(timeRemaining);
      else
         return computeSecondSegmentProjection(doubleSupportDurations, singleSupportDurations, timeRemaining);
   }

   private double computeFirstSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState)
   {
      double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
      double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

      double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

      double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;
      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;

      double recursionTime = timeInState + upcomingInitialDoubleSupportDuration + endOfDoubleSupportDuration - stepDuration;
      double recursion = Math.exp(omega.getDoubleValue() * recursionTime);

      return 1.0 / recursion;
   }

   private double computeSecondSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining)
   {
      swingStateEndRecursionMatrix.computeInSingleSupport(doubleSupportDurations, singleSupportDurations, startOfSplineTime.getDoubleValue(),
            endOfSplineTime.getDoubleValue());

      double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
      double timeRemainingInSpline = timeRemaining - lastSegmentDuration;
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicProjectionDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicProjectionDerivativeMatrix.update(timeRemainingInSpline);
      cubicProjectionMatrix.setSegmentDuration(splineDuration);
      cubicProjectionMatrix.update(timeRemainingInSpline);

      CommonOps.mult(cubicProjectionMatrix, swingStateEndRecursionMatrix, matrixOut);

      return 1.0 / matrixOut.get(0, 0);
   }

   private double computeThirdSegmentProjection(double timeRemaining)
   {
      return computeInSwingOneCMP(timeRemaining);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicProjectionDerivativeMatrix, transferStateEndRecursionMatrix, matrixOut);

      return 1.0 / matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMPVelocity()
   {
      return omega.getDoubleValue() * positionMultiplier.getDoubleValue();
   }

   private double computeSegmentedVelocityProjection(double timeRemaining)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentVelocityProjection();
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentVelocityProjection();
      else
         return computeSecondSegmentVelocityProjection();
   }

   private double computeFirstSegmentVelocityProjection()
   {
      return omega.getDoubleValue() * positionMultiplier.getDoubleValue();
   }

   private double computeSecondSegmentVelocityProjection()
   {
      CommonOps.mult(cubicProjectionDerivativeMatrix, swingStateEndRecursionMatrix, matrixOut);

      return 1.0 / matrixOut.get(0, 0);
   }

   private double computeThirdSegmentVelocityProjection()
   {
      return omega.getDoubleValue() * positionMultiplier.getDoubleValue();
   }
}
