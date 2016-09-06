package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class StateEndRecursionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPRatio;

   public StateEndRecursionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio, DoubleYoVariable exitCMPRatio)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPRatio = exitCMPRatio;
   }

   public void reset()
   {
      zero();
   }

   public void computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations)
   {
      double stateDuration = doubleSupportDurations.get(0).getDoubleValue();

      double stateRecursion = Math.exp(-omega.getDoubleValue() * stateDuration);

      set(0, 0, stateRecursion);
      set(1, 0, omega.getDoubleValue() * stateRecursion);
      set(2, 0, 1.0);
      set(3, 0, omega.getDoubleValue());
   }

   public void computeInTransfer(double doubleSupportDuration)
   {
      double stateRecursion = Math.exp(-omega.getDoubleValue() * doubleSupportDuration);

      set(0, 0, stateRecursion);
      set(1, 0, omega.getDoubleValue() * stateRecursion);
      set(2, 0, 1.0);
      set(3, 0, omega.getDoubleValue());
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
      double lastSegmentDuration = stepDuration - endOfSplineTime;

      double endOfCurrentDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * currentDoubleSupportDuration;
      double upcomingInitialDoubleSupportDuration = doubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;

      double timeSpentOnExitCMP = exitCMPRatio.getDoubleValue() * stepDuration;
      double timeSpentOnEntryCMP = (1.0 - exitCMPRatio.getDoubleValue()) * stepDuration;

      double recursionTimeToCorner = upcomingInitialDoubleSupportDuration - timeSpentOnExitCMP;
      double recursionTimeToStart = recursionTimeToCorner - timeSpentOnEntryCMP + endOfCurrentDoubleSupportDuration + startOfSplineTime;

      double stateRecursionToStart = Math.exp(omega.getDoubleValue() * recursionTimeToStart);
      double stateRecursionToEnd = Math.exp(-omega.getDoubleValue() * lastSegmentDuration);

      set(0, 0, stateRecursionToStart);
      set(1, 0, omega.getDoubleValue() * stateRecursionToStart);
      set(2, 0, stateRecursionToEnd);
      set(3, 0, omega.getDoubleValue() * stateRecursionToEnd);
   }
}

