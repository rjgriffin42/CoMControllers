package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.interpolation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.MathTools;

public class CubicProjectionMatrix extends DenseMatrix64F
{
   private final CubicSplineMatrix cubicSplineMatrix = new CubicSplineMatrix();
   private final CubicTimeMatrix cubicTimeMatrix = new CubicTimeMatrix();

   private double duration;

   public CubicProjectionMatrix()
   {
      super(1, 4);
   }

   public void setSegmentDuration(double duration)
   {
      this.duration = duration;
      cubicSplineMatrix.setSegmentDuration(duration);
   }

   public void update(double timeInCurrentState)
   {
      timeInCurrentState = MathTools.clipToMinMax(timeInCurrentState, 0.0, duration);
      cubicTimeMatrix.setCurrentTime(timeInCurrentState);

      CommonOps.mult(cubicTimeMatrix, cubicSplineMatrix, this);
   }
}
