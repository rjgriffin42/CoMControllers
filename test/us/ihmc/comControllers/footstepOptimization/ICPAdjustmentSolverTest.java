package us.ihmc.comControllers.footstepOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.comControllers.icpOptimization.FootstepRecursionMultiplierCalculator;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Vector2d;
import java.util.ArrayList;

public class ICPAdjustmentSolverTest extends ICPAdjustmentSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry("registry");
   private final DoubleYoVariable omega = new DoubleYoVariable("omega", registry);

   private final DoubleYoVariable yoDoubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable yoSingleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("exitCMPDurationInPercentOfStepTime", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("doubleSupportSplitFraction", registry);

   private final TargetTouchdownICPCalculator targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega, registry);
   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(yoDoubleSupportDuration,
         yoSingleSupportDuration, exitCMPDurationInPercentOfStepTime, doubleSupportSplitFraction, omega, maxNumberOfFootstepsToConsider, registry);

   private static final double epsilon = 0.00001;
   private static final int maxNumberOfFootstepsToConsider = 5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FramePoint2d perfectCMP = new FramePoint2d(worldFrame);
   private final FramePoint2d currentICP = new FramePoint2d(worldFrame);
   private final FramePoint2d targetTouchdownICP = new FramePoint2d(worldFrame);
   private final FramePoint2d desiredFinalICP = new FramePoint2d(worldFrame);
   private final FramePoint2d finalICPRecursion = new FramePoint2d(worldFrame);

   private final FrameVector2d entryOffset = new FrameVector2d(worldFrame);
   private final FrameVector2d exitOffset = new FrameVector2d(worldFrame);

   private final FramePoint2d twoCMPOffsetEffect = new FramePoint2d(worldFrame);

   private final ArrayList<FramePoint2d> desiredFootsteps = new ArrayList<>();

   private static final double footstepWeight = 100.0;
   private static final double feedbackWeight = 2.0;
   private static final double feedbackGain = 2.0;

   private static final double stepLength = 0.2;
   private static final double stanceWidth = 0.1;

   private static final double singleSupportDuration = 0.6;
   private static final double doubleSupportDuration = 0.2;
   private static final double remainingTime = 0.4;
   private static final double steppingDuration = singleSupportDuration + doubleSupportDuration;

   private static final double timeSpentOnExitCMPRatio = 0.5;
   private static final double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;
   private static final double totalTimeSpentOnEntryCMP = (1.0 - timeSpentOnExitCMPRatio) * steppingDuration;


   public ICPAdjustmentSolverTest()
   {

      super( new ICPAdjustmentControllerParameters()
      {
         @Override public int getMaximumNumberOfStepsToConsider()
         {
            return 5;
         }

         @Override public int getNumberOfStepsToConsider()
         {
            return 3;
         }

         @Override public double getFootstepWeight()
         {
            return 1.5;
         }

         @Override public double getFeedbackWeight()
         {
            return 2.0;
         }

         @Override public boolean useFeedback()
         {
            return false;
         }

         @Override public boolean scaleFirstStepWithTime()
         {
            return false;
         }

         @Override public double minimumRemainingTime()
         {
            return 0.001;
         }

         @Override public double minimumFootstepWeight()
         {
            return 0.0001;
         }

         @Override public double minimumFeedbackWeight()
         {
            return 0.0001;
         }
      });

      for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.add(new FramePoint2d(worldFrame));

      omega.set(3.0);
      entryOffset.setX(-0.05);
      exitOffset.setX(0.05);

      yoDoubleSupportDuration.set(doubleSupportDuration);
      yoSingleSupportDuration.set(singleSupportDuration);
      exitCMPDurationInPercentOfStepTime.set(0.5);
      doubleSupportSplitFraction.set(0.5);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDimensionality()
   {
      int numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider - 2;

      super.setProblemConditions(numberOfFootstepsToConsider, true, true, true);
      checkDimensions(numberOfFootstepsToConsider, true, true, true);

      super.setProblemConditions(numberOfFootstepsToConsider, false, true, true);
      checkDimensions(numberOfFootstepsToConsider, false, true, true);

      super.setProblemConditions(numberOfFootstepsToConsider, false, true, false);
      checkDimensions(numberOfFootstepsToConsider, false, true, false);

      super.setProblemConditions(numberOfFootstepsToConsider, true, false, true);
      checkDimensions(numberOfFootstepsToConsider, true, false, true);

      super.setProblemConditions(numberOfFootstepsToConsider, true, false, false);
      checkDimensions(numberOfFootstepsToConsider, true, false, false);

      super.setProblemConditions(numberOfFootstepsToConsider, true, true, false);
      checkDimensions(numberOfFootstepsToConsider, true, true, false);



      numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider + 2;

      super.setProblemConditions(numberOfFootstepsToConsider, true, true, true);
      checkDimensions(maxNumberOfFootstepsToConsider, true, true, true);

      super.setProblemConditions(numberOfFootstepsToConsider, false, true, true);
      checkDimensions(maxNumberOfFootstepsToConsider, false, true, true);

      super.setProblemConditions(numberOfFootstepsToConsider, false, true, false);
      checkDimensions(maxNumberOfFootstepsToConsider, false, true, false);

      super.setProblemConditions(numberOfFootstepsToConsider, true, false, true);
      checkDimensions(maxNumberOfFootstepsToConsider, true, false, true);

      super.setProblemConditions(numberOfFootstepsToConsider, true, false, false);
      checkDimensions(maxNumberOfFootstepsToConsider, true, false, false);

      super.setProblemConditions(numberOfFootstepsToConsider, true, true, false);
      checkDimensions(maxNumberOfFootstepsToConsider, true, true, false);
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testPerfectOneCMPNoFeedback()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
         runStepTestPerfectNoFeedbackOneCMP(1);
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testPerfectOneCMPWithFeedback()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
         runStepTestPerfectWithFeedbackOneCMP(1);
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testPerfectTwoCMPsNoFeedback()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
         runStepTestPerfectNoFeedbackTwoCMPs(1);
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testPerfectTwoCMPsWithFeedback()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
         runStepTestPerfectWithFeedbackTwoCMPs(1);
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testStepNoFeedbackOneCMP()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 1; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            runStepTestNoFeedbackOneCMP(i);
         }
      }
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testStepOnlyFeedbackOneCMP()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 1; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            runStepTestOnlyFeedbackOneCMP(i);
         }
      }
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testStepOneCMPWithFeedback()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 1; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            runStepTestOneCMPWithFeedback(i);
         }
      }
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testStepNoFeedbackTwoCMPs()
   {
      int iters = 1;

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 1; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            runStepTestNoFeedbackTwoCMPs(i);
         }
      }
   }

   @DeployableTestMethod(estimatedDuration = 2.0)
   @Test(timeout = 21000)
   public void testStepTwoCMPsWithFeedback()
   {
      int iters = 1;

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 1; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            runStepTestTwoCMPsWithFeedback(i);
         }
      }
   }

   private void runStepTestPerfectNoFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      final boolean useStepAdjustment = true;
      final boolean useFeedback = false;
      final boolean useTwoCMPs = false;

      setPerfectOneCMPConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);

      FramePoint2d footstep1 = new FramePoint2d(worldFrame);
      FramePoint2d feedback = new FramePoint2d(worldFrame);
      FrameVector2d feedbackDelta = new FrameVector2d(worldFrame);

      super.getFootstepSolutionLocation(0, footstep1);
      super.getCMPFeedback(feedback);
      super.getCMPFeedbackDifference(feedbackDelta);

      JUnitTools.assertTuple2dEquals("", desiredFootsteps.get(0).getPoint(), footstep1.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", perfectCMP.getPoint(), feedback.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", new Vector2d(), feedbackDelta.getVector(), epsilon);
   }


   private void runStepTestPerfectWithFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = true;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = false;

      setPerfectOneCMPConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);

      FramePoint2d footstep1 = new FramePoint2d(worldFrame);
      FramePoint2d feedback = new FramePoint2d(worldFrame);
      FrameVector2d feedbackDelta = new FrameVector2d(worldFrame);

      super.getFootstepSolutionLocation(0, footstep1);
      super.getCMPFeedback(feedback);
      super.getCMPFeedbackDifference(feedbackDelta);

      JUnitTools.assertTuple2dEquals("", desiredFootsteps.get(0).getPoint(), footstep1.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", perfectCMP.getPoint(), feedback.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", new Vector2d(), feedbackDelta.getVector(), epsilon);
   }

   private void runStepTestPerfectNoFeedbackTwoCMPs(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = false;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = true;

      setPerfectTwoCMPsConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);

      FramePoint2d footstep1 = new FramePoint2d(worldFrame);
      FramePoint2d feedback = new FramePoint2d(worldFrame);
      FrameVector2d feedbackDelta = new FrameVector2d(worldFrame);

      super.getFootstepSolutionLocation(0, footstep1);
      super.getCMPFeedback(feedback);
      super.getCMPFeedbackDifference(feedbackDelta);

      JUnitTools.assertTuple2dEquals("", desiredFootsteps.get(0).getPoint(), footstep1.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", perfectCMP.getPoint(), feedback.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", new Vector2d(), feedbackDelta.getVector(), epsilon);
   }

   private void runStepTestPerfectWithFeedbackTwoCMPs(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = true;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = true;

      setPerfectTwoCMPsConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);

      FramePoint2d footstep1 = new FramePoint2d(worldFrame);
      FramePoint2d feedback = new FramePoint2d(worldFrame);
      FrameVector2d feedbackDelta = new FrameVector2d(worldFrame);

      super.getFootstepSolutionLocation(0, footstep1);
      super.getCMPFeedback(feedback);
      super.getCMPFeedbackDifference(feedbackDelta);

      JUnitTools.assertTuple2dEquals("", desiredFootsteps.get(0).getPoint(), footstep1.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", perfectCMP.getPoint(), feedback.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", new Vector2d(), feedbackDelta.getVector(), epsilon);
   }

   private void runStepTestNoFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = false;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = false;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestOnlyFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = true;
      final boolean useStepAdjustment = false;
      final boolean useTwoCMPs = false;

      numberOfFootstepsToConsider = 0;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestOneCMPWithFeedback(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = true;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = false;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestTwoCMPsWithFeedback(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = true;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = true;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestNoFeedbackTwoCMPs(int numberOfFootstepsToConsider)
   {
      final boolean useFeedback = false;
      final boolean useStepAdjustment = true;
      final boolean useTwoCMPs = true;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, useFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, useFeedback, useStepAdjustment, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void setPerfectOneCMPConditions(int numberOfFootstepsToConsider, boolean useFeedback, boolean useStepAdjustment, boolean useTwoCMPs)
   {
      perfectCMP.setToZero(worldFrame);
      currentICP.setToZero(worldFrame);

      double remainingTime = this.remainingTime;
      if (!useStepAdjustment)
         remainingTime = 1 / omega.getDoubleValue() * Math.log( feedbackGain / ( 1 + feedbackGain ) );

      for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.get(i).setToZero();

      targetTouchdownICP.setToZero(worldFrame);
      desiredFinalICP.setToZero(worldFrame);
      finalICPRecursion.setToZero(worldFrame);

      RobotSide stanceSide = RobotSide.RIGHT;

      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
      {
         desiredFootsteps.get(i).set((i + 1) * stepLength, stanceSide.negateIfRightSide(stanceWidth));
         stanceSide = stanceSide.getOppositeSide();
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));
      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, false, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

      FramePoint2d perfectHeelCMP = new FramePoint2d();
      FramePoint2d perfectTargetHeelStrikeICP = new FramePoint2d();
      perfectTargetHeelStrikeICP.set(finalICPRecursion);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         perfectHeelCMP.set(desiredFootsteps.get(i));
         perfectHeelCMP.scale(footstepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs));

         perfectTargetHeelStrikeICP.add(perfectHeelCMP);
      }

      perfectCMP.set(0.0, -stanceWidth); // right foot stance

      currentICP.set(perfectTargetHeelStrikeICP);
      currentICP.scale(Math.exp(-omega.getDoubleValue() * remainingTime));
      perfectHeelCMP.set(perfectCMP);
      perfectHeelCMP.scale(1 - Math.exp(-omega.getDoubleValue() * remainingTime));
      currentICP.add(perfectHeelCMP);

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));
   }

   private void setPerfectTwoCMPsConditions(int numberOfFootstepsToConsider, boolean useFeedback, boolean useTwoCMPs)
   {
      perfectCMP.setToZero(worldFrame);
      currentICP.setToZero(worldFrame);

      for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.get(i).setToZero();

      targetTouchdownICP.setToZero(worldFrame);
      desiredFinalICP.setToZero(worldFrame);
      finalICPRecursion.setToZero(worldFrame);

      RobotSide stanceSide = RobotSide.RIGHT;

      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
      {
         desiredFootsteps.get(i).set((i + 1) * stepLength, stanceSide.negateIfRightSide(stanceWidth));
         stanceSide = stanceSide.getOppositeSide();
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));
      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, false, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

      FramePoint2d perfectExitCMP = new FramePoint2d();
      FramePoint2d perfectEntryCMP = new FramePoint2d();
      FramePoint2d perfectTargetHeelStrikeICP = new FramePoint2d();
      perfectTargetHeelStrikeICP.set(finalICPRecursion);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         perfectExitCMP.set(desiredFootsteps.get(i));
         perfectExitCMP.add(exitOffset);
         perfectExitCMP.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs));

         perfectTargetHeelStrikeICP.add(perfectExitCMP);

         perfectEntryCMP.set(desiredFootsteps.get(i));
         perfectEntryCMP.add(entryOffset);
         perfectEntryCMP.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs));

         perfectTargetHeelStrikeICP.add(perfectEntryCMP);
      }

      perfectCMP.set(0.0, -stanceWidth); // right foot stance
      perfectCMP.add(exitOffset); // right foot stance

      currentICP.set(perfectTargetHeelStrikeICP);
      currentICP.scale(Math.exp(-omega.getDoubleValue() * remainingTime));
      perfectExitCMP.set(perfectCMP);
      perfectExitCMP.scale(1 - Math.exp(-omega.getDoubleValue() * remainingTime));
      currentICP.add(perfectExitCMP);

      twoCMPOffsetEffect.setToZero();

      if (useTwoCMPs)
      {
         FramePoint2d totalCMPOffset = new FramePoint2d(worldFrame);
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            totalCMPOffset.set(exitOffset);
            totalCMPOffset.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);

            totalCMPOffset.set(entryOffset);
            totalCMPOffset.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);
         }
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));
   }

   private void setImperfectConditions(int numberOfFootstepsToConsider)
   {
      perfectCMP.setToZero(worldFrame);
      currentICP.setToZero(worldFrame);

      for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.get(i).setToZero();

      targetTouchdownICP.setToZero(worldFrame);
      desiredFinalICP.setToZero(worldFrame);
      finalICPRecursion.setToZero(worldFrame);

      perfectCMP.set(0.0, -stanceWidth); // right foot stance
      currentICP.set(0.1, 0.0);
      RobotSide stanceSide = RobotSide.RIGHT;

      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
      {
         desiredFootsteps.get(i).set((i + 1) * stepLength, stanceSide.negateIfRightSide(stanceWidth));
         stanceSide = stanceSide.getOppositeSide();
      }

      twoCMPOffsetEffect.setToZero();

      if (useTwoCMPs)
      {
         FramePoint2d totalCMPOffset = new FramePoint2d(worldFrame);
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            totalCMPOffset.set(exitOffset);
            totalCMPOffset.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);

            totalCMPOffset.set(entryOffset);
            totalCMPOffset.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);
         }
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));
   }

   private void submitConditions(int numberOfFootstepsToConsider, boolean useFeedback, boolean useTwoCMPs)
   {
      double feedbackDynamicEffect = (1.0 - Math.exp(omega.getDoubleValue() * remainingTime));

      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);

      if (useTwoCMPs)
         footstepRecursionMultiplierCalculator
               .computeRecursionMultipliers(numberOfFootstepsToConsider, false, useTwoCMPs);
      else
         footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider, false, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

      if (useTwoCMPs)
         finalICPRecursion.add(twoCMPOffsetEffect);

      if (useFeedback)
         submitInformation(feedbackDynamicEffect, numberOfFootstepsToConsider, useTwoCMPs);
      else
         submitInformation(numberOfFootstepsToConsider, useTwoCMPs);

   }

   private void submitInformation(double feedbackDynamicEffect, int numberOfFootstepsToConsider, boolean useTwoCMPs)
   {
      submitInformation(numberOfFootstepsToConsider, useTwoCMPs);

      super.setFeedbackWeight(feedbackWeight);
      super.setFeedbackDynamicEffect(feedbackDynamicEffect);
   }

   private void submitInformation(int numberOfFootstepsToConsider, final boolean useTwoCMPs)
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         super.setReferenceFootstepLocation(i, desiredFootsteps.get(i));

      super.setPerfectCMP(perfectCMP);
      super.setTargetTouchdownICP(targetTouchdownICP);
      super.setFinalICPRecursion(finalICPRecursion);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         double recursionMultiplier;
         if (useTwoCMPs)
         {
            double entryRecursionMultiplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs);
            double exitRecursionMultiplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);

            recursionMultiplier = entryRecursionMultiplier + exitRecursionMultiplier;
         }
         else
         {
            recursionMultiplier = footstepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs);
         }

         super.setFootstepRecursionMultipliers(i, recursionMultiplier);
         super.setFootstepWeight(i, footstepWeight);
      }
   }

   private void checkMatrices(int numberOfFootstepsToConsider, boolean useFeedback, boolean useStepAdjustment, final boolean useTwoCMPs)
   {
      if (!useStepAdjustment)
         numberOfFootstepsToConsider = 0;

      DenseMatrix64F identity = CommonOps.identity(2, 2);
      DenseMatrix64F dynamicsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         double recursionMultiplier;
         if (useTwoCMPs)
            recursionMultiplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs) + footstepRecursionMultiplierCalculator
                  .getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);
         else
            recursionMultiplier = footstepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs);

         CommonOps.setIdentity(identity);
         CommonOps.scale(recursionMultiplier, identity);
         CommonOps.extract(tmpDynamics_Aeq, 2 * i, 2 * i + 2, 0, 2, dynamicsSubset, 0, 0);

         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, dynamicsSubset, identity, epsilon);
      }

      DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);
      if (numberOfFootstepsToConsider > 0)
      {
         rightHandSide.set(0, 0, targetTouchdownICP.getX() - finalICPRecursion.getX());
         rightHandSide.set(1, 0, targetTouchdownICP.getY() - finalICPRecursion.getY());
      }
      else
      {
         rightHandSide.set(0, 0, targetTouchdownICP.getX());
         rightHandSide.set(1, 0, targetTouchdownICP.getY());
      }
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, rightHandSide, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, solverInput_Aeq, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, solverInput_beq, epsilon);

      int size = 2 * numberOfFootstepsToConsider;
      DenseMatrix64F zeros;
      if (useFeedback)
         zeros = new DenseMatrix64F(size + 2, 1);
      else
         zeros = new DenseMatrix64F(size, 1);

      int optimizationSize;
      if (useFeedback)
         optimizationSize = 2 * numberOfFootstepsToConsider + 2;
      else
         optimizationSize = 2 * numberOfFootstepsToConsider;

      DenseMatrix64F optimizationBlock = new DenseMatrix64F(optimizationSize, optimizationSize);
      DenseMatrix64F optimizationEquals = new DenseMatrix64F(optimizationSize, 1);
      CommonOps.extract(solverInput_G, 0, optimizationSize, 0, optimizationSize, optimizationBlock, 0, 0);
      CommonOps.extract(solverInput_g, 0, optimizationSize, 0, 1, optimizationEquals, 0, 0);

      if (useFeedback)
      {
         DenseMatrix64F feedbackBlock = new DenseMatrix64F(2, 2);
         DenseMatrix64F feedbackEquals = new DenseMatrix64F(2, 1);
         CommonOps.extract(solverInput_G, size + 2, size + 4, size + 2, size + 4, feedbackBlock, 0, 0);
         CommonOps.extract(solverInput_g, size + 2, size + 4, 0, 1, feedbackEquals, 0, 0);

         size += 2;
      }

      DenseMatrix64F constraintBlock = new DenseMatrix64F(size, 2);
      DenseMatrix64F constraintEquals = new DenseMatrix64F(2, 1);
      CommonOps.extract(solverInput_G, 0, size, size, size + 2, constraintBlock, 0, 0);
      CommonOps.extract(solverInput_g, size, size + 2, 0, 1, constraintEquals, 0, 0);

      DenseMatrix64F dynamicsConstraintBlock = new DenseMatrix64F(size, 2);
      DenseMatrix64F dynamicsConstraintEqual = new DenseMatrix64F(2, 1);
      CommonOps.extract(solverInput_Aeq, 0, size, 0, 2, dynamicsConstraintBlock, 0, 0);
      CommonOps.extract(solverInput_beq, 0, 2, 0, 1, dynamicsConstraintEqual, 0, 0);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, dynamicsConstraintBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, dynamicsConstraintEqual, epsilon);

      DenseMatrix64F weightsSubset = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepWeights = CommonOps.identity(2, 2);
      DenseMatrix64F feedbackWeights = CommonOps.identity(2, 2);

      CommonOps.scale(footstepWeight, footstepWeights);
      CommonOps.scale(feedbackWeight, feedbackWeights);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         int start = 2 * i;
         CommonOps.extract(solverInput_H, start, start + 2, start, start + 2, weightsSubset, 0, 0);
         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, weightsSubset, footstepWeights, epsilon);
      }

      if (useFeedback)
      {
         int start = 2 * numberOfFootstepsToConsider;
         CommonOps.extract(solverInput_H, start, start + 2, start, start + 2, weightsSubset, 0, 0);
         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, weightsSubset, feedbackWeights, epsilon);
      }

      // check that the cost was inserted into total problem correctly
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, solverInput_H, optimizationBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, solverInput_h, optimizationEquals, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, constraintBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, constraintEquals, epsilon);
   }

   private void checkSolutions(int numberOfFootstepsToConsider)
   {
      Assert.assertTrue("Considering " + numberOfFootstepsToConsider + " steps cannot solve.", !MathTools.containsNaN(solution));
      Assert.assertTrue("step " + numberOfFootstepsToConsider, super.getCostToGo() >= -epsilon);

      FramePoint2d footstepLocation = new FramePoint2d();
      DenseMatrix64F solutionBlock = new DenseMatrix64F(2, 1);
      DenseMatrix64F readSolution = new DenseMatrix64F(2, 1);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         super.getFootstepSolutionLocation(i, footstepLocation);

         CommonOps.extract(solution, 2 * i, 2 * (i + 1), 0, 1, solutionBlock, 0, 0);
         readSolution.set(0, 0, footstepLocation.getX());
         readSolution.set(1, 0, footstepLocation.getY());

         JUnitTools.assertMatrixEquals("step " + i, solutionBlock, readSolution, epsilon);
      }

      if (useFeedback)
      {
         FramePoint2d feedbackLocation = new FramePoint2d();

         FramePoint2d feedbackDeltaSolution = new FramePoint2d();
         FramePoint2d feedbackSolution = new FramePoint2d();

         super.getCMPFeedback(feedbackLocation);

         CommonOps.extract(solution, 2 * numberOfFootstepsToConsider, 2 * (numberOfFootstepsToConsider + 1), 0, 1, solutionBlock, 0, 0);

         feedbackDeltaSolution.set(solutionBlock.get(0, 0), solutionBlock.get(1, 0));
         feedbackSolution.set(perfectCMP);
         feedbackSolution.add(feedbackDeltaSolution);

         JUnitTools.assertPoint2dEquals("feedback", feedbackLocation.getPoint(), feedbackSolution.getPoint(), epsilon);
      }
   }

   private void checkDimensions(int numberOFFootstepsToConsider, boolean useFeedback, boolean useStepAdjustment, boolean useTwoCMPs)
   {
      int totalFreeVariables, totalFootstepVariables;

      int totalLagrangeMultipliers = 2;

      if (useStepAdjustment)
         totalFootstepVariables = 2 * numberOFFootstepsToConsider;
      else
         totalFootstepVariables = 0;

      if (useFeedback)
         totalFreeVariables = totalFootstepVariables + 2;
      else
         totalFreeVariables = totalFootstepVariables;

      if (useStepAdjustment)
         Assert.assertEquals("", numberOFFootstepsToConsider, this.numberOfFootstepsToConsider, epsilon);
      else
         Assert.assertEquals("", 0, this.numberOfFootstepsToConsider, epsilon);
      Assert.assertEquals("", totalFreeVariables, this.totalFreeVariables, epsilon);
      Assert.assertEquals("", totalFootstepVariables, this.totalFootstepVariables, epsilon);
      Assert.assertEquals("", totalLagrangeMultipliers, this.totalLagrangeMultipliers, epsilon);
      Assert.assertEquals("", useFeedback, this.useFeedback);
      Assert.assertEquals("", useTwoCMPs, this.useTwoCMPs);
   }
}
