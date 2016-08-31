package us.ihmc.comControllers.icpOptimization;

import org.junit.Assert;
import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class ICPOptimizationSolverTest extends ICPOptimizationSolver
{
   private static final YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRobert");

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.0001;

   public ICPOptimizationSolverTest()
   {
      super(icpOptimizationParameters, rootRegistry);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testSetFeedbackConditions()
   {
      Random random = new Random();
      double feedbackWeight = 10.0 * random.nextDouble();
      double feedbackGain = 10.0 * random.nextDouble();

      super.setFeedbackConditions(feedbackWeight, feedbackGain);

      Assert.assertEquals("", feedbackWeight, this.feedbackWeight.get(0, 0), epsilon);
      Assert.assertEquals("", feedbackWeight, this.feedbackWeight.get(1, 1), epsilon);

      Assert.assertEquals("", feedbackGain, this.feedbackGain.get(1, 1), epsilon);
      Assert.assertEquals("", feedbackGain, this.feedbackGain.get(1, 1), epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testDimensions()
   {
      for (int i = 1; i < this.maximumNumberOfFootstepsToConsider; i++)
      {
         testDimension(i, false, true, false);
         testDimension(i, true, true, false);
         testDimension(i, true, false, false);
      }
   }

   public void testDimension(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      super.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs);

      double numberOfLagrangeMultipliers = 2;
      double numberOfFootstepVariables = 0;
      if (useStepAdjustment)
         numberOfFootstepVariables = 2 * numberOfFootstepsToConsider;

      double totalNumberOfFreeVariables = numberOfFootstepVariables;

      if (useFeedback)
         totalNumberOfFreeVariables += 2;

      String name = "Number of Steps: " + numberOfFootstepsToConsider + ". Use step adjustment: " + useStepAdjustment + ". Use Feedback: " + useFeedback;
      Assert.assertEquals(name, numberOfFootstepVariables, this.numberOfFootstepVariables, epsilon);
      Assert.assertEquals(name, numberOfLagrangeMultipliers, this.numberOfLagrangeMultipliers, epsilon);
      Assert.assertEquals(name, totalNumberOfFreeVariables, this.numberOfFreeVariables, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 2000)
   public void testConditionError()
   {
      testCondition(0, true, false, true);
      testCondition(0, true, false, false);
      testCondition(0, false, false, true);
      testCondition(0, false, false, false);
      testCondition(1, false, false, true);
      testCondition(1, false, false, false);
   }

   public void testCondition(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      boolean hasError = false;
      try
      {
         super.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs);
      }
      catch (RuntimeException e)
      {
         hasError = true;
      }

      Assert.assertTrue(hasError);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDynamicConstraint()
   {
      double omega = 3.0;
      double remainingTime = 0.5;

      double doubleSupportDuration = 0.2;
      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      int numberOffootstepsToConsider = 0;
      super.submitProblemConditions(numberOffootstepsToConsider, true, true, false);
      super.setFeedbackConditions(2.0, 0.001);

      double finalICPRecursionMultiplier = Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d finalICP = new FramePoint2d(worldFrame, 0.2, 0.115);
      FramePoint2d finalICPRecursion = new FramePoint2d();
      finalICPRecursion.set(finalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      double cmpProjectionMultiplier = Math.exp(omega * remainingTime) - Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.0, -0.155);
      FramePoint2d cmpProjection = new FramePoint2d();
      cmpProjection.set(perfectCMP);
      cmpProjection.scale(cmpProjectionMultiplier);

      FramePoint2d currentICP = new FramePoint2d(worldFrame, 0.1, 0.06);

      compute(finalICPRecursion, null, currentICP, perfectCMP, cmpProjection, omega, timeRemaining);

      FramePoint2d rightHandSide = new FramePoint2d();
      rightHandSide.set(currentICP);
      rightHandSide.scale(Math.exp(omega * timeRemaining));
      rightHandSide.sub(cmpProjection);
      rightHandSide.sub(finalICPRecursion);

      DenseMatrix64F expectedDynamics_beq = new DenseMatrix64F(2, 1);
      expectedDynamics_beq.set(0, 0, rightHandSide.getX());
      expectedDynamics_beq.set(1, 0, rightHandSide.getY());

      JUnitTools.assertMatrixEquals(expectedDynamics_beq, dynamics_beq, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStanding()
   {
      DoubleYoVariable omega = new DoubleYoVariable("omega", rootRegistry);
      omega.set(3.0);

      FramePoint2d icpDesired = new FramePoint2d();
      FrameVector2d icpDesiredVelocity = new FrameVector2d();
      FramePoint2d icpActual = new FramePoint2d();
      icpActual.set(0.05, 0.07);

      FramePoint2d perfectCMP = new FramePoint2d();
      CapturePointTools.computeDesiredCentroidalMomentumPivot(icpDesired, icpDesiredVelocity, omega.getDoubleValue(), perfectCMP);

      FramePoint2d cmpDesired = new FramePoint2d();

      FramePoint2d cmpGoal = new FramePoint2d();
      cmpGoal.set(icpActual);
      cmpGoal.sub(icpDesired);
      cmpGoal.scale(icpOptimizationParameters.getFeedbackGain());
      cmpGoal.add(perfectCMP);

      ICPOptimizationController icpOptimizationController = new ICPOptimizationController(icpPlannerParameters, icpOptimizationParameters, null, null, omega, rootRegistry);
      icpOptimizationController.initializeForStanding(0.0);

      icpOptimizationController.compute(0.5, icpDesired, icpDesiredVelocity, icpActual);
      icpOptimizationController.getDesiredCMP(cmpDesired);

      JUnitTools.assertPoint2dEquals("", cmpGoal.getPoint(), cmpDesired.getPoint(), epsilon);
   }

   private static final ICPOptimizationParameters icpOptimizationParameters = new ICPOptimizationParameters()
   {
      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override
      public int numberOfFootstepsToConsider()
      {
         return 1;
      }

      @Override public double getFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFeedbackWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackGain()
      {
         return 2.0;
      }

      @Override public boolean scaleFirstStepWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean useFeedback()
      {
         return true;
      }

      @Override public boolean useStepAdjustment()
      {
         return false;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumTimeRemaining()
      {
         return 0.001;
      }
   };

   private static final CapturePointPlannerParameters icpPlannerParameters = new CapturePointPlannerParameters()
   {
      @Override public double getDoubleSupportInitialTransferDuration()
      {
         return 1.0;
      }

      @Override public double getEntryCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getExitCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public boolean useTwoCMPsPerSupport()
      {
         return false;
      }

      @Override public double getMaxEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMaxExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinExitCMPForwardOffset()
      {
         return 0;
      }
   };
}
