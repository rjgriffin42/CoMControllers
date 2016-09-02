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

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   public ContinuousCurrentStateProjectionMultiplier(YoVariableRegistry registry, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio)
   {
      super(registry);

      this.omega = omega;
      this.stateEndRecursionMatrix = new StateEndRecursionMatrix(omega, doubleSupportSplitRatio);
      this.cubicProjectionMatrix = new CubicProjectionMatrix();
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                       boolean useTwoCMPs, boolean isInTransfer)
   {
      if (useTwoCMPs || isInTransfer)
      {
         stateEndRecursionMatrix.compute(doubleSupportDurations, singleSupportDurations, useTwoCMPs, isInTransfer);
         if (isInTransfer)
            this.compute(timeRemaining, doubleSupportDurations.get(0).getDoubleValue());
         else
            this.compute(timeRemaining, singleSupportDurations.get(0).getDoubleValue());
      }
      else
      {
         this.set(Math.exp(omega.getDoubleValue() * timeRemaining));
      }
   }

   public void compute(double timeRemaining, double duration)
   {
      cubicProjectionMatrix.setSegmentDuration(duration);
      cubicProjectionMatrix.update(timeRemaining);
      CommonOps.mult(cubicProjectionMatrix, stateEndRecursionMatrix, matrixOut);

      this.set(1.0 / matrixOut.get(0, 0));
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
