package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class SwingStateEndRecursionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;

   private final DoubleYoVariable doubleSupportSplitRatio;

   public SwingStateEndRecursionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio)
   {
      super(4, 1);

      this.omega = omega;

      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
   }

   public void reset()
   {
      zero();
   }

   public void computeInSingleSupport(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
                                      double startOfSplineTime, double endOfSplineTime)
   {
      double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

      computeInSingleSupport(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDurations.get(0).getDoubleValue(), startOfSplineTime, endOfSplineTime);
   }

   public void computeInSingleSupport(double upcomingDoubleSupportDuration, double currentDoubleSupportDuration, double singleSupportDuration,
                                      double startOfSplineTime, double endOfSplineTime)
   {
      double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

      double endOfCurrentDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * currentDoubleSupportDuration;
      double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;

      double lastSegmentDuration = stepDuration - endOfSplineTime;
      double stateRecursionToEnd = Math.exp(-omega.getDoubleValue() * lastSegmentDuration);

      double recursionTimeToInitial = upcomingInitialDoubleSupportDuration + endOfCurrentDoubleSupportDuration + startOfSplineTime - stepDuration;
      double stateRecursionToStart = Math.exp(omega.getDoubleValue() * recursionTimeToInitial);

      set(0, 0, stateRecursionToStart);
      set(1, 0, omega.getDoubleValue() * stateRecursionToStart);
      set(2, 0, stateRecursionToEnd);
      set(3, 0, omega.getDoubleValue() * stateRecursionToEnd);
   }
}

