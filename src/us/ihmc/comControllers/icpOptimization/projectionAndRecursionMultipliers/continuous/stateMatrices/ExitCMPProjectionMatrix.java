package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class ExitCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   public ExitCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
                                  DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, boolean useTwoCMPs, boolean isInTransfer)
   {
      double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();
      compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDurations.get(0).getDoubleValue(), useTwoCMPs, isInTransfer);
   }

   public void compute(double upcomingDoubleSupportDuration, double currentDoubleSupportDuration, double singleSupportDuration, boolean useTwoCMPs, boolean isInTransfer)
   {
      double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

      double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;
      double currentInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * currentDoubleSupportDuration;
      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;

      double initialDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * currentInitialDoubleSupportDuration);
      double endOfDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * endOfDoubleSupportDuration);

      zero();

      if (useTwoCMPs)
      {
         if (!isInTransfer)
         {
            double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
            double lastSegmentRecursion = Math.exp(-omega.getDoubleValue() * lastSegmentDuration);

            double timeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * stepDuration;
            double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * stepDuration;

            double exitCMPTime = endOfDoubleSupportDuration - timeSpentOnEntryCMP;
            double exitCMPRecursion = Math.exp(omega.getDoubleValue() * exitCMPTime);

            double entryCMPTime = upcomingInitialDoubleSupportDuration - timeSpentOnExitCMP;
            double entryCMPRecursion = 1.0 - Math.exp(omega.getDoubleValue() * entryCMPTime);

            double stepProjection = exitCMPRecursion * entryCMPRecursion;

            set(0, 0, stepProjection);
            set(1, 0, omega.getDoubleValue() * stepProjection);
            set(2, 0, 1.0 - lastSegmentRecursion);
            set(3, 0, -omega.getDoubleValue() * lastSegmentRecursion);
         }
      }
      else
      {
         if (isInTransfer)
         {
            double totalProjection = initialDoubleSupportProjection * (1.0 - endOfDoubleSupportProjection);

            set(0, 0, totalProjection);
            set(1, 0, omega.getDoubleValue() * totalProjection);
         }
         else
         {
            double stepProjection = Math.exp(-omega.getDoubleValue() * singleSupportDuration);

            set(0, 0, 1.0 - stepProjection);
            set(1, 0, -omega.getDoubleValue() * stepProjection);
         }

         set(3, 0, -omega.getDoubleValue());
      }
   }
}
