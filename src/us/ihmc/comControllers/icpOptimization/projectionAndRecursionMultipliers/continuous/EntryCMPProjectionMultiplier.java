package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation.CubicProjectionDerivativeMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation.CubicProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.swing.SwingExitCMPProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.transfer.TransferEntryCMPProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.transfer.TransferExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class EntryCMPProjectionMultiplier
{
   private final CubicProjectionMatrix cubicProjectionMatrix;
   private final CubicProjectionDerivativeMatrix cubicProjectionDerivativeMatrix;

   private final TransferEntryCMPProjectionMatrix transferEntryCMPProjectionMatrix;
   private final SwingExitCMPProjectionMatrix swingEntryCMPProjectionMatrix;

   private final DoubleYoVariable omega;
   private final DoubleYoVariable exitCMPRatio;
   private final DoubleYoVariable doubleSupportSplitRatio;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public EntryCMPProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio,
         DoubleYoVariable exitCMPRatio, DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      positionMultiplier = new DoubleYoVariable("EntryCMPProjectionMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("EntryCMPVelocityProjectionMultiplier", registry);

      this.omega = omega;
      this.exitCMPRatio = exitCMPRatio;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;

      cubicProjectionMatrix = new CubicProjectionMatrix();
      cubicProjectionDerivativeMatrix = new CubicProjectionDerivativeMatrix();

      transferEntryCMPProjectionMatrix = new TransferEntryCMPProjectionMatrix(omega, doubleSupportSplitRatio);
      swingEntryCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPRatio, startOfSplineTime, endOfSplineTime, totalTrajectoryTime);
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
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeRemaining, useTwoCMPs);
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
            positionMultiplier = computeInSwingOneCMP();
            velocityMultiplier = computeInSwingOneCMPVelocity();
         }
      }

      this.positionMultiplier.set(positionMultiplier);
      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining, boolean useTwoCMPs)
   {
      transferEntryCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs);

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicProjectionDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicProjectionDerivativeMatrix.update(timeRemaining);
      cubicProjectionMatrix.setSegmentDuration(splineDuration);
      cubicProjectionMatrix.update(timeRemaining);

      CommonOps.mult(cubicProjectionMatrix, transferEntryCMPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMP()
   {
      return 0.0;
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicProjectionDerivativeMatrix, transferEntryCMPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInSwingOneCMPVelocity()
   {
      return 0.0;
   }

   private double computeSegmentedProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining)
   {
      double timeInState = totalTrajectoryTime.getDoubleValue() - timeRemaining;

      if (timeInState < startOfSplineTime.getDoubleValue())
         return computeFirstSegmentProjection(doubleSupportDurations, singleSupportDurations, timeInState);
      else if (timeInState >= endOfSplineTime.getDoubleValue())
         return computeThirdSegmentProjection();
      else
         return computeSecondSegmentProjection(doubleSupportDurations, singleSupportDurations, timeRemaining);
   }

   private double computeFirstSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeInState)
   {
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
      double singleSupportDuration = singleSupportDurations.get(0).getDoubleValue();

      double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

      double timeSpentOnEntryCMP = (1.0 - exitCMPRatio.getDoubleValue()) * stepDuration;

      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;

      double entryRecursionTime = timeInState + endOfDoubleSupportDuration - timeSpentOnEntryCMP;
      double entryRecursion = Math.exp(omega.getDoubleValue() * entryRecursionTime);

      double recursion = 1.0 - entryRecursion;

      return recursion;
   }

   private double computeSecondSegmentProjection(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         double timeRemaining)
   {
      swingEntryCMPProjectionMatrix.compute(doubleSupportDurations, singleSupportDurations);

      double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
      double timeRemainingInSpline = timeRemaining - lastSegmentDuration;
      double splineDuration = endOfSplineTime.getDoubleValue() - startOfSplineTime.getDoubleValue();

      cubicProjectionMatrix.setSegmentDuration(splineDuration);
      cubicProjectionMatrix.update(timeRemainingInSpline);

      cubicProjectionDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicProjectionDerivativeMatrix.update(timeRemainingInSpline);

      CommonOps.mult(cubicProjectionMatrix, swingEntryCMPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeThirdSegmentProjection()
   {
      return computeInSwingOneCMP();
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
      return omega.getDoubleValue() * (positionMultiplier.getDoubleValue() - 1.0);
   }

   private double computeSecondSegmentVelocityProjection()
   {
      CommonOps.mult(cubicProjectionDerivativeMatrix, swingEntryCMPProjectionMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeThirdSegmentVelocityProjection()
   {
      return omega.getDoubleValue() * computeInSwingOneCMPVelocity();
   }
}
