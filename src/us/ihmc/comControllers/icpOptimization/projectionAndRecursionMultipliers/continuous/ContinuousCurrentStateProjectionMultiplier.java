package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.CurrentStateProjectionMultiplier;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation.CubicProjectionMatrix;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.StateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ContinuousCurrentStateProjectionMultiplier extends CurrentStateProjectionMultiplier
{
   private final StateEndRecursionMatrix stateEndRecursionMatrix;
   private final CubicProjectionMatrix cubicProjectionMatrix;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   public ContinuousCurrentStateProjectionMultiplier(StateEndRecursionMatrix stateEndRecursionMatrix, CubicProjectionMatrix cubicProjectionMatrix, YoVariableRegistry registry, DoubleYoVariable omega)
   {
      super(registry);

      this.stateEndRecursionMatrix = stateEndRecursionMatrix;
      this.cubicProjectionMatrix = cubicProjectionMatrix;
   }

   public void compute(double timeRemaining)
   {
      cubicProjectionMatrix.update(timeRemaining);
      CommonOps.mult(cubicProjectionMatrix, stateEndRecursionMatrix, matrixOut);

      this.set(1.0 / matrixOut.get(0, 0));
   }

   public void reset()
   {
      this.set(0.0);
   }
}
