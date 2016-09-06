package us.ihmc.comControllers.icpOptimization;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.comControllers.icpOptimization.FootstepRecursionMultiplierCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
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

   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(null, exitCMPDurationInPercentOfStepTime,
         doubleSupportSplitFraction, omega, maximumNumberOfStepsToConsider, registry);

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
   public void testCalculatingTwoCMPsSingleSupport()
   {
      int stepsToConsider = 2;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMPRatio = 0.5;
      double totalTimeSpentOnEntryCMP = (1.0 - timeSpentOnExitCMPRatio) * steppingDuration;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;

      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;


      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(doubleSupportSplitFraction);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, false, true);

      double totalTime = initialDoubleSupportDuration + stepsToConsider * (doubleSupportDuration + singleSupportDuration);
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("Final ICP Recursion Multiplier", finalICPRecursionMultiplier, footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double multiplier = Math.exp(-omega.getDoubleValue() * (initialDoubleSupportDuration + i * steppingDuration));
         double desiredExitRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         desiredExitRecursionMultiplier = multiplier * desiredExitRecursionMultiplier;
         double exitRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(i);

         double desiredEntryRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);
         desiredEntryRecursionMultiplier = multiplier * desiredEntryRecursionMultiplier;
         double entryRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(i);

         Assert.assertEquals("Step " + i + " Exit Recursion Multiplier", desiredExitRecursionMultiplier, exitRecursionMultiplier, epsilon);
         Assert.assertEquals("Step " + i + " Entry Recursion Multiplier", desiredEntryRecursionMultiplier, entryRecursionMultiplier, epsilon);
      }

      double timeRemaining = 0.5 * singleSupportDuration;
      footstepRecursionMultiplierCalculator.computeRemainingProjectionMultipliers(timeRemaining, true, false, false);

      double entryStanceProjectionMultiplier = footstepRecursionMultiplierCalculator.getStanceEntryCMPProjectionMultiplier();
      double exitStanceProjectionMultiplier = footstepRecursionMultiplierCalculator.getStanceExitCMPProjectionMultiplier();

      double desiredStanceExitProjectionMutliplier = Math.exp(omega.getDoubleValue() * timeRemaining) - Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);
      double desiredStanceEntryProjectionMutliplier = 0.0;

      Assert.assertEquals("Exit Stance Projection Multiplier", desiredStanceExitProjectionMutliplier, exitStanceProjectionMultiplier, epsilon);
      Assert.assertEquals("Entry Stance Projection Multiplier", desiredStanceEntryProjectionMutliplier, entryStanceProjectionMultiplier, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculatingTwoCMPsTranfser()
   {
      int stepsToConsider = 2;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMPRatio = 0.5;
      double totalTimeSpentOnEntryCMP = (1.0 - timeSpentOnExitCMPRatio) * steppingDuration;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;

      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;
      double endDoubleSupportDuration = (1.0 - doubleSupportSplitFraction) * doubleSupportDuration;

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(doubleSupportSplitFraction);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, true, true);

      double totalTime = (stepsToConsider + 1) * (doubleSupportDuration + singleSupportDuration) - endDoubleSupportDuration;
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("Final ICP Recursion Multiplier", finalICPRecursionMultiplier, footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double multiplierTime = initialDoubleSupportDuration - endDoubleSupportDuration + (i + 1) * steppingDuration;
         double multiplier = Math.exp(-omega.getDoubleValue() * multiplierTime);

         double desiredExitRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         desiredExitRecursionMultiplier = multiplier * desiredExitRecursionMultiplier;
         double exitRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(i);

         double desiredEntryRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);
         desiredEntryRecursionMultiplier = multiplier * desiredEntryRecursionMultiplier;
         double entryRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(i);

         Assert.assertEquals("Step " + i + " Exit Recursion Multiplier", desiredExitRecursionMultiplier, exitRecursionMultiplier, epsilon);
         Assert.assertEquals("Step " + i + " Entry Recursion Multiplier", desiredEntryRecursionMultiplier, entryRecursionMultiplier, epsilon);
      }

      double timeRemaining = 0.5 * doubleSupportDuration;
      footstepRecursionMultiplierCalculator.computeRemainingProjectionMultipliers(timeRemaining, true, true, false);

      double entryStanceProjectionMultiplier = footstepRecursionMultiplierCalculator.getStanceEntryCMPProjectionMultiplier();
      double exitStanceProjectionMultiplier = footstepRecursionMultiplierCalculator.getStanceExitCMPProjectionMultiplier();

      double desiredStanceEntryProjectionMutliplier = Math.exp(omega.getDoubleValue() * timeRemaining) - Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - endDoubleSupportDuration));
      double desiredStanceExitProjectionMutliplier = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);
      desiredStanceExitProjectionMutliplier = desiredStanceExitProjectionMutliplier * Math.exp(-omega.getDoubleValue() * (totalTimeSpentOnEntryCMP - endDoubleSupportDuration));

      Assert.assertEquals("Exit Stance Projection Multiplier", desiredStanceExitProjectionMutliplier, exitStanceProjectionMultiplier, epsilon);
      Assert.assertEquals("Entry Stance Projection Multiplier", desiredStanceEntryProjectionMutliplier, entryStanceProjectionMultiplier, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculatingOneCMPSingleSupport()
   {
      int stepsToConsider = 2;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMPRatio = 0.5;

      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(doubleSupportSplitFraction);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, false, false);

      double totalTime = initialDoubleSupportDuration + stepsToConsider * steppingDuration;
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("", finalICPRecursionMultiplier, footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double desiredRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * steppingDuration);
         desiredRecursionMultiplier = desiredRecursionMultiplier * Math.exp(-omega.getDoubleValue() * (initialDoubleSupportDuration + i * steppingDuration));
         double recursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(i);

         Assert.assertEquals("Step " + i, desiredRecursionMultiplier, recursionMultiplier, epsilon);
      }

      double timeRemaining = 0.5 * singleSupportDuration;
      footstepRecursionMultiplierCalculator.computeRemainingProjectionMultipliers(timeRemaining, false, false, false);
      double stanceProjectionMultiplier = footstepRecursionMultiplierCalculator.getStanceExitCMPProjectionMultiplier();
      double desiredStanceProjectionMutliplier = Math.exp(omega.getDoubleValue() * timeRemaining) - Math.exp(-omega.getDoubleValue() * initialDoubleSupportDuration);

      Assert.assertEquals("", desiredStanceProjectionMutliplier, stanceProjectionMultiplier, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculatingOneCMPTransfer()
   {
      int stepsToConsider = 2;

      omega.set(3.0);

      double doubleSupportDuration = 0.5;
      double singleSupportDuration = 1.5;
      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double timeSpentOnExitCMPRatio = 0.5;

      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      this.exitCMPDurationInPercentOfStepTime.set(timeSpentOnExitCMPRatio);
      this.doubleSupportSplitFraction.set(doubleSupportSplitFraction);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(stepsToConsider, true, false);

      double totalTime = initialDoubleSupportDuration + singleSupportDuration + stepsToConsider * steppingDuration;
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("", finalICPRecursionMultiplier, footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double desiredRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * steppingDuration);
         desiredRecursionMultiplier = desiredRecursionMultiplier * Math.exp(-omega.getDoubleValue() * (initialDoubleSupportDuration + singleSupportDuration + i * steppingDuration));
         double recursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(i);

         Assert.assertEquals("Step " + i, desiredRecursionMultiplier, recursionMultiplier, epsilon);
      }

      double timeRemaining = 0.5 * doubleSupportDuration;
      footstepRecursionMultiplierCalculator.computeRemainingProjectionMultipliers(timeRemaining, false, true, false);
      double stanceProjectionMultiplier = footstepRecursionMultiplierCalculator.getStanceExitCMPProjectionMultiplier();
      double desiredStanceProjectionMutliplier = Math.exp(omega.getDoubleValue() * timeRemaining) - Math.exp(-omega.getDoubleValue() * (initialDoubleSupportDuration + singleSupportDuration));

      Assert.assertEquals("", desiredStanceProjectionMutliplier, stanceProjectionMultiplier, epsilon);
   }
}
