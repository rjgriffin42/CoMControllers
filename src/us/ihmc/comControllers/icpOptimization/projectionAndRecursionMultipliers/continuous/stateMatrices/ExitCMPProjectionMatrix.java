package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class ExitCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public ExitCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio, DoubleYoVariable exitCMPDurationInPercentOfStepTime)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, boolean useTwoCMPs, boolean isInTransfer)
   {
      double stepDuration = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();

      double initialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDurations.get(0).getDoubleValue();
      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double initialDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);
      double endOfDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * endOfDoubleSupportDuration);

      zero();

      if (useTwoCMPs)
      {
         if (!isInTransfer)
         {
            double timeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * stepDuration;
            double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * stepDuration;

            double stepProjection = Math.exp(omega.getDoubleValue() * (endOfDoubleSupportDuration - timeSpentOnEntryCMP)) *
                  (1.0 - Math.exp(omega.getDoubleValue() * (initialDoubleSupportDuration - timeSpentOnExitCMP)));

            set(0, 0, stepProjection);
            set(1, 0, omega.getDoubleValue() * stepProjection);
            set(3, 0, -omega.getDoubleValue());
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
            double stepProjection = Math.exp(-omega.getDoubleValue() * stepDuration);

            set(0, 0, 1.0 - stepProjection);
            set(1, 0, -omega.getDoubleValue() * stepProjection);
         }

         set(3, 0, -omega.getDoubleValue());
      }

   }
}
