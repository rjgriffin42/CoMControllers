package us.ihmc.comControllers.footstepOptimization;

public interface ICPAdjustmentControllerParameters
{
   public int getMaximumNumberOfStepsToConsider();

   public int getNumberOfStepsToConsider();

   public double getFootstepWeight();

   public double getFeedbackWeight();

   public boolean useFeedback();

   public boolean scaleFirstStepWithTime();

   public double minimumRemainingTime();

   public double minimumFootstepWeight();

   public double minimumFeedbackWeight();
}
