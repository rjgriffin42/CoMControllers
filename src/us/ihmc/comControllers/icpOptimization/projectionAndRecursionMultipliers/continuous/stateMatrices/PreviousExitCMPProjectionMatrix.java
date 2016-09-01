package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class PreviousExitCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;

   public PreviousExitCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, boolean isInTransfer)
   {
      compute(doubleSupportDurations.get(0).getDoubleValue(), isInTransfer);
   }

   public void compute(double doubleSupportDuration, boolean isInTransfer)
   {
      if (isInTransfer)
      {
         double initialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * doubleSupportDuration;
         double initialDoubleSupportProjection = Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);

         zero();
         set(0, 0, 1.0 - initialDoubleSupportProjection);
         set(1, 0, -omega.getDoubleValue() * initialDoubleSupportProjection);
      }
      else
      {
         zero();
      }
   }
}
