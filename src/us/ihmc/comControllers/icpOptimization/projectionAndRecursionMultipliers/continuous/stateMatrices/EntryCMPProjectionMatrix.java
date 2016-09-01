package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class EntryCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public EntryCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio, DoubleYoVariable exitCMPDurationInPercentOfStepTime)
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
      this.compute(doubleSupportDurations.get(0).getDoubleValue(), singleSupportDurations.get(0).getDoubleValue(), useTwoCMPs, isInTransfer);
   }

   public void compute(double doubleSupportDuration, double singleSupportDuration, boolean useTwoCMPs, boolean isInTransfer)
   {
      zero();

      if (useTwoCMPs)
      {
         double stepDuration = doubleSupportDuration + singleSupportDuration;

         double initialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDuration;
         double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDuration;

         double initialDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);
         double endOfDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * endOfDoubleSupportDuration);

         if (isInTransfer)
         {
            double stepProjection = initialDoubleSupportProjection * (1.0 - endOfDoubleSupportProjection);

            set(0, 0, stepProjection);
            set(1, 0, omega.getDoubleValue() * stepProjection);
            set(3, 0, -omega.getDoubleValue());
         }
         else
         {
            double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * stepDuration;

            double duration = endOfDoubleSupportDuration - timeSpentOnEntryCMP;
            double projection = Math.exp(omega.getDoubleValue() * duration);

            set(0, 0, 1.0 - projection);
            set(1, 0, -omega.getDoubleValue() * projection);
         }
      }

   }

}
