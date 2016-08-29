package us.ihmc.comControllers.generatedTestSuites;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.comControllers.icpOptimization.FootstepRecursionMultiplierCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FootstepRecursionMultiplierCalculatorTest
{
   private static final int maximumNumberOfStepsToConsider = 5;
   private final YoVariableRegistry registry = new YoVariableRegistry("robert");
   private final DoubleYoVariable omega = new DoubleYoVariable("omega", registry);

   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("timeSpentOnExitCMPInPercentOfStepTime", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("doubleSupportSplitFraction", registry);

   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(doubleSupportDuration,
         singleSupportDuration, exitCMPDurationInPercentOfStepTime, doubleSupportSplitFraction, omega, maximumNumberOfStepsToConsider, registry);

   private final double epsilon = 0.0001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testErrorsInCalculatingTwoCMPTooManySteps()
   {
      int stepsToConsider = 6;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double timeSpentOnExitCMPRatio = 0.5;

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(0.5);

      boolean pass = false;
      try
      {
         footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, false, true);
      }
      catch(RuntimeException e)
      {
         pass = true;
      }

      Assert.assertTrue(pass);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculatingTwoCMPs()
   {
      int stepsToConsider = 2;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMPRatio = 0.5;
      double totalTimeSpentOnEntryCMP = (1.0 - timeSpentOnExitCMPRatio) * steppingDuration;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(0.5);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, false, true);

      double totalTime = stepsToConsider * (doubleSupportDuration + singleSupportDuration);
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("", finalICPRecursionMultiplier, footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double multiplier = Math.exp(-omega.getDoubleValue() * i * steppingDuration);
         double desiredExitRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         desiredExitRecursionMultiplier = multiplier * desiredExitRecursionMultiplier;
         double exitRecursionMultiplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, true);

         double desiredEntryRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);
         desiredEntryRecursionMultiplier = multiplier * desiredEntryRecursionMultiplier;
         double entryRecursionMultiplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, true);

         Assert.assertEquals("Step " + i, desiredExitRecursionMultiplier, exitRecursionMultiplier, epsilon);
         Assert.assertEquals("Step " + i, desiredEntryRecursionMultiplier, entryRecursionMultiplier, epsilon);
      }
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculatingOneCMPs()
   {
      int stepsToConsider = 2;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMPRatio = 0.5;

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(0.5);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, false, false);

      double totalTime = stepsToConsider * steppingDuration;
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("", finalICPRecursionMultiplier, footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double desiredRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * steppingDuration);
         desiredRecursionMultiplier = desiredRecursionMultiplier * Math.exp(-omega.getDoubleValue() * i * steppingDuration);
         double recursionMultiplier = footstepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, false);


         Assert.assertEquals("Step " + i, desiredRecursionMultiplier, recursionMultiplier, epsilon);
      }
   }
}
