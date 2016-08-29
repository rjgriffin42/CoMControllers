package us.ihmc.comControllers.icpOptimization;

public interface ICPOptimizationParameters
{
   public int getMaximumNumberOfFootstepsToConsider();

   public int numberOfFootstepsToConsider();

   public double getFootstepWeight();

   public double getFeedbackWeight();

   public double getFeedbackGain();

   public boolean scaleFirstStepWeightWithTime();

   public boolean scaleFeedbackWeightWithGain();

   public boolean useFeedback();

   public boolean useStepAdjustment();

   public double getMinimumFootstepWeight();

   public double getMinimumFeedbackWeight();

   public double getMinimumTimeRemaining();
}
