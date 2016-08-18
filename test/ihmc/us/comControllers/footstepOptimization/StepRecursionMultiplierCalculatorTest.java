package ihmc.us.comControllers.footstepOptimization;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class StepRecursionMultiplierCalculatorTest
{
   private final double epsilon = 0.0001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testErrorsInCalculatingOneCMP()
   {
      int maxNumberOfStepsToConsider = 5;
      int stepsToConsider = 3;

      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      double steppingDuration = 2.0;

      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfStepsToConsider, registry);

      boolean pass = false;
      try
      {
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, stepsToConsider, true);
      }
      catch(RuntimeException e)
      {
         pass = true;
      }

      Assert.assertTrue(pass);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testErrorsInCalculatingTwoCMP()
   {
      int maxNumberOfStepsToConsider = 5;
      int stepsToConsider = 3;


      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      double steppingDuration = 2.0;
      double timeSpentOnExitCMPRatio = 0.5;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;
      double totalTimeSpentOnEntryCMP = (1 - timeSpentOnExitCMPRatio) * steppingDuration;

      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfStepsToConsider, registry);

      boolean pass = false;
      try
      {
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, stepsToConsider, false);
      }
      catch(RuntimeException e)
      {
         pass = true;
      }

      Assert.assertTrue(pass);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testErrorsInCalculatingOneCMPTooManySteps()
   {
      int maxNumberOfStepsToConsider = 5;
      int stepsToConsider = 6;

      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      double steppingDuration = 2.0;

      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfStepsToConsider, registry);

      boolean pass = false;
      try
      {
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, stepsToConsider, false);
      }
      catch(RuntimeException e)
      {
         pass = true;
      }

      Assert.assertTrue(pass);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testErrorsInCalculatingTwoCMPTooManySteps()
   {
      int maxNumberOfStepsToConsider = 5;
      int stepsToConsider = 6;


      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      double steppingDuration = 2.0;
      double timeSpentOnExitCMPRatio = 0.5;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;
      double totalTimeSpentOnEntryCMP = (1 - timeSpentOnExitCMPRatio) * steppingDuration;

      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfStepsToConsider, registry);

      boolean pass = false;
      try
      {
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, stepsToConsider, true);
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
      int maxNumberOfStepsToConsider = 5;
      int stepsToConsider = 2;

      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      double steppingDuration = 2.0;
      double timeSpentOnExitCMPRatio = 0.3;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;
      double totalTimeSpentOnEntryCMP = (1 - timeSpentOnExitCMPRatio) * steppingDuration;

      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfStepsToConsider, registry);

      stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, stepsToConsider, true);

      double totalTime = stepsToConsider * steppingDuration;
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("", finalICPRecursionMultiplier, stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double multiplier = Math.exp(-omega.getDoubleValue() * i * steppingDuration);
         double desiredExitRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP));
         desiredExitRecursionMultiplier = multiplier * desiredExitRecursionMultiplier;
         double exitRecursionMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, true);

         double desiredEntryRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnEntryCMP);
         desiredEntryRecursionMultiplier = multiplier * desiredEntryRecursionMultiplier;
         double entryRecursionMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, true);

         Assert.assertEquals("Step " + i, desiredExitRecursionMultiplier, exitRecursionMultiplier, epsilon);
         Assert.assertEquals("Step " + i, desiredEntryRecursionMultiplier, entryRecursionMultiplier, epsilon);
      }
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculatingOneCMPs()
   {
      int maxNumberOfStepsToConsider = 5;
      int stepsToConsider = 2;

      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      double steppingDuration = 2.0;

      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfStepsToConsider, registry);

      stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, stepsToConsider, false);

      double totalTime = stepsToConsider * steppingDuration;
      double finalICPRecursionMultiplier = Math.exp(-omega.getDoubleValue() * totalTime);

      Assert.assertEquals("", finalICPRecursionMultiplier, stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier(), epsilon);

      for (int i = 0; i < stepsToConsider; i++)
      {
         double desiredRecursionMultiplier = 1.0 - Math.exp(-omega.getDoubleValue() * steppingDuration);
         desiredRecursionMultiplier = desiredRecursionMultiplier * Math.exp(-omega.getDoubleValue() * i * steppingDuration);
         double recursionMultiplier = stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, false);


         Assert.assertEquals("Step " + i, desiredRecursionMultiplier, recursionMultiplier, epsilon);
      }
   }
}
