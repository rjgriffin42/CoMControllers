package ihmc.us.comControllers.footstepOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point2d;
import java.util.ArrayList;
import java.util.Random;

public class ICPAdjustmentSolverTest extends ICPAdjustmentSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry("registry");
   private final DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
   private final TargetTouchdownICPCalculator targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega, registry);
   private final StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfFootstepsToConsider, registry);

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

   private static final double footstepWeight = 3.0;
   private static final double feedbackWeight = 1.5;
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
      super(maxNumberOfFootstepsToConsider);

      for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.add(new FramePoint2d(worldFrame));

      omega.set(3.0);
      entryOffset.setX(-0.05);
      exitOffset.setX(0.05);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDimensionality()
   {
      int numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider - 2;

      super.setProblemConditions(numberOfFootstepsToConsider, false, false);
      checkDimensions(numberOfFootstepsToConsider, false, false);

      super.setProblemConditions(numberOfFootstepsToConsider, true, false);
      checkDimensions(numberOfFootstepsToConsider, true, false);

      super.setProblemConditions(numberOfFootstepsToConsider, false, true);
      checkDimensions(numberOfFootstepsToConsider, false, true);

      super.setProblemConditions(numberOfFootstepsToConsider, true, true);
      checkDimensions(numberOfFootstepsToConsider, true, true);


      numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider + 2;

      super.setProblemConditions(numberOfFootstepsToConsider, false, false);
      checkDimensions(maxNumberOfFootstepsToConsider, false, false);

      super.setProblemConditions(numberOfFootstepsToConsider, true, false);
      checkDimensions(maxNumberOfFootstepsToConsider, true, false);

      super.setProblemConditions(numberOfFootstepsToConsider, false, true);
      checkDimensions(maxNumberOfFootstepsToConsider, false, true);

      super.setProblemConditions(numberOfFootstepsToConsider, true, true);
      checkDimensions(maxNumberOfFootstepsToConsider, true, true);
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
   public void testStepNoFeedbackOneCMP()
   {
      int iters = 100;

      for (int iter = 0; iter < iters; iter++)
      {
         for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            Random random = new Random();
            int numberOfFootstepsToConsider = random.nextInt();
            numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider);
            numberOfFootstepsToConsider = Math.max(numberOfFootstepsToConsider, 1);

            runStepTestNoFeedbackOneCMP(numberOfFootstepsToConsider);
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
         for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            Random random = new Random();
            int numberOfFootstepsToConsider = random.nextInt();
            numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider);
            numberOfFootstepsToConsider = Math.max(numberOfFootstepsToConsider, 1);

            runStepTestOneCMPWithFeedback(numberOfFootstepsToConsider);
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
         for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            Random random = new Random();
            int numberOfFootstepsToConsider = random.nextInt();
            numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider);
            numberOfFootstepsToConsider = Math.max(numberOfFootstepsToConsider, 1);

            runStepTestNoFeedbackTwoCMPs(numberOfFootstepsToConsider);
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
         for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
         {
            Random random = new Random();
            int numberOfFootstepsToConsider = random.nextInt();
            numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider);
            numberOfFootstepsToConsider = Math.max(numberOfFootstepsToConsider, 1);

            runStepTestTwoCMPsWithFeedback(numberOfFootstepsToConsider);
         }
      }
   }

   private void runStepTestPerfectNoFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      final boolean includeFeedback = false;
      final boolean useTwoCMPs = false;

      setPerfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);

      FramePoint2d footstep1 = new FramePoint2d(worldFrame);
      FramePoint2d feedback = new FramePoint2d(worldFrame);

      super.getFootstepSolutionLocation(0, footstep1);
      super.getCMPFeedback(feedback);

      JUnitTools.assertTuple2dEquals("", desiredFootsteps.get(0).getPoint(), footstep1.getPoint(), epsilon);
      JUnitTools.assertTuple2dEquals("", perfectCMP.getPoint(), feedback.getPoint(), epsilon);
   }

   private void runStepTestNoFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      final boolean includeFeedback = false;
      final boolean useTwoCMPs = false;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestOneCMPWithFeedback(int numberOfFootstepsToConsider)
   {
      final boolean includeFeedback = true;
      final boolean useTwoCMPs = false;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestTwoCMPsWithFeedback(int numberOfFootstepsToConsider)
   {
      final boolean includeFeedback = true;
      final boolean useTwoCMPs = true;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void runStepTestNoFeedbackTwoCMPs(int numberOfFootstepsToConsider)
   {
      final boolean includeFeedback = false;
      final boolean useTwoCMPs = true;

      setImperfectConditions(numberOfFootstepsToConsider);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      submitConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatrices(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.solve();

      checkSolutions(numberOfFootstepsToConsider);
   }

   private void setPerfectConditions(int numberOfFootstepsToConsider)
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

      if (useTwoCMPs)
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, numberOfFootstepsToConsider, useTwoCMPs);
      else
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, numberOfFootstepsToConsider, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

      FramePoint2d perfectTargetHeelStrikeICP = new FramePoint2d();
      FramePoint2d perfectHeelCMP = new FramePoint2d();
      perfectTargetHeelStrikeICP.set(finalICPRecursion);
      perfectTargetHeelStrikeICP.set(finalICPRecursion);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         perfectHeelCMP.set(desiredFootsteps.get(i));
         perfectHeelCMP.scale(stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs));

         perfectTargetHeelStrikeICP.add(perfectHeelCMP);
      }

      perfectCMP.set(0.0, -stanceWidth); // right foot stance

      currentICP.set(perfectTargetHeelStrikeICP);
      currentICP.scale(Math.exp(-omega.getDoubleValue() * remainingTime));
      perfectHeelCMP.set(perfectCMP);
      perfectHeelCMP.scale(1 - Math.exp(-omega.getDoubleValue() * remainingTime));
      currentICP.add(perfectHeelCMP);

      twoCMPOffsetEffect.setToZero();

      if (useTwoCMPs)
      {
         FramePoint2d totalCMPOffset = new FramePoint2d(worldFrame);
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            totalCMPOffset.set(exitOffset);
            totalCMPOffset.scale(stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);

            totalCMPOffset.set(entryOffset);
            totalCMPOffset.scale(stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs));

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
            totalCMPOffset.scale(stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);

            totalCMPOffset.set(entryOffset);
            totalCMPOffset.scale(stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs));

            twoCMPOffsetEffect.add(totalCMPOffset);
         }
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));
   }

   private void submitConditions(int numberOfFootstepsToConsider, boolean includeFeedback, boolean useTwoCMPs)
   {
      double effectiveFeedbackGain = -Math.exp(omega.getDoubleValue() * remainingTime) / (1.0 + feedbackGain / omega.getDoubleValue());

      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);

      if (useTwoCMPs)
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, numberOfFootstepsToConsider, useTwoCMPs);
      else
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, numberOfFootstepsToConsider, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

      if (useTwoCMPs)
         finalICPRecursion.add(twoCMPOffsetEffect);

      if (includeFeedback)
         submitInformation(effectiveFeedbackGain, numberOfFootstepsToConsider, useTwoCMPs);
      else
         submitInformation(numberOfFootstepsToConsider, useTwoCMPs);

   }

   private void submitInformation(double effectiveFeedbackWeight, int numberOfFootstepsToConsider, boolean useTwoCMPs)
   {
      submitInformation(numberOfFootstepsToConsider, useTwoCMPs);

      super.setFeedbackWeight(feedbackWeight);
      super.setEffectiveFeedbackGain(effectiveFeedbackWeight);
   }

   private void submitInformation(int numberOfFootstepsToConsider, final boolean useTwoCMPs)
   {
      if (entryOffset != null && exitOffset != null)
      {
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
            super.setReferenceFootstepLocation(i, desiredFootsteps.get(i), entryOffset, exitOffset);
      }
      else
      {
         for (int i = 0; i < numberOfFootstepsToConsider; i++)
            super.setReferenceFootstepLocation(i, desiredFootsteps.get(i));
      }

      super.setPerfectCMP(perfectCMP);
      super.setTargetTouchdownICP(targetTouchdownICP);
      super.setFinalICPRecursion(finalICPRecursion);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         double recursionMultiplier;
         if (useTwoCMPs)
         {
            double entryRecursionMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs);
            double exitRecursionMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);

            recursionMultiplier = entryRecursionMultiplier + exitRecursionMultiplier;
         }
         else
         {
            recursionMultiplier = stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs);
         }

         super.setFootstepRecursionMultipliers(i, recursionMultiplier);
         super.setFootstepWeight(i, footstepWeight);
      }
   }

   private void checkMatrices(int numberOfFootstepsToConsider, boolean includeFeedback, final boolean useTwoCMPs)
   {
      DenseMatrix64F identity = CommonOps.identity(2, 2);
      DenseMatrix64F dynamicsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         double recursionMultiplier;
         if (useTwoCMPs)
            recursionMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs) + stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);
         else
            recursionMultiplier = stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs);

         CommonOps.setIdentity(identity);
         CommonOps.scale(recursionMultiplier, identity);
         CommonOps.extract(tmpDynamics_Aeq, 2 * i, 2 * i + 2, 0, 2, dynamicsSubset, 0, 0);

         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, dynamicsSubset, identity, epsilon);
      }

      DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);
      rightHandSide.set(0, 0, targetTouchdownICP.getX() - finalICPRecursion.getX());
      rightHandSide.set(1, 0, targetTouchdownICP.getY() - finalICPRecursion.getY());
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, rightHandSide, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, solverInput_Aeq, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, solverInput_beq, epsilon);

      int size = 2 * numberOfFootstepsToConsider;
      DenseMatrix64F zeros;
      if (includeFeedback)
         zeros = new DenseMatrix64F(size + 2, 1);
      else
         zeros = new DenseMatrix64F(size, 1);

      int optimizationSize;
      if (includeFeedback)
         optimizationSize = 2 * numberOfFootstepsToConsider + 2;
      else
         optimizationSize = 2 * numberOfFootstepsToConsider;

      DenseMatrix64F optimizationBlock = new DenseMatrix64F(optimizationSize, optimizationSize);
      DenseMatrix64F optimizationEquals = new DenseMatrix64F(optimizationSize, 1);
      CommonOps.extract(solverInput_G, 0, optimizationSize, 0, optimizationSize, optimizationBlock, 0, 0);
      CommonOps.extract(solverInput_g, 0, optimizationSize, 0, 1, optimizationEquals, 0, 0);

      if (includeFeedback)
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

      if (includeFeedback)
      {
         int start = 2 * numberOfFootstepsToConsider;
         CommonOps.extract(solverInput_H, start, start + 2, start, start + 2, weightsSubset, 0, 0);
         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, weightsSubset, feedbackWeights, epsilon);
      }

      // check that the cost was inserted into total problem correctly
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, solverInput_H, optimizationBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, zeros, optimizationEquals, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, constraintBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, constraintEquals, epsilon);
   }

   private void checkSolutions(int numberOfFootstepsToConsider)
   {
      Assert.assertTrue(super.getCostToGo() > 0.0);
      Assert.assertTrue(!MathTools.containsNaN(solution));

      FramePoint2d footstepLocation = new FramePoint2d();
      FramePoint2d feedbackLocation = new FramePoint2d();
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

      if (includeFeedback)
      {
         super.getCMPFeedback(feedbackLocation);

         CommonOps.extract(solution, 2 * numberOfFootstepsToConsider, 2 * (numberOfFootstepsToConsider + 1), 0, 0, solutionBlock, 0, 0);
         readSolution.set(0, 0, feedbackLocation.getX());
         readSolution.set(1, 0, feedbackLocation.getY());

         JUnitTools.assertMatrixEquals("feedback", solutionBlock, readSolution, epsilon);
      }
   }

   private void checkDimensions(int numberOFFootstepsToConsider, boolean includeFeedback, boolean useTwoCMPs)
   {
      int totalFreeVariables;

      int totalFootstepVariables = 2 * numberOFFootstepsToConsider;
      int totalLagrangeMultipliers = 2;

      if (includeFeedback)
         totalFreeVariables = totalFootstepVariables + 2;
      else
         totalFreeVariables = totalFootstepVariables;

      Assert.assertEquals("", numberOfFootstepsToConsider, this.numberOfFootstepsToConsider, epsilon);
      Assert.assertEquals("", totalFreeVariables, this.totalFreeVariables, epsilon);
      Assert.assertEquals("", totalFootstepVariables, this.totalFootstepVariables, epsilon);
      Assert.assertEquals("", totalLagrangeMultipliers, this.totalLagrangeMultipliers, epsilon);
      Assert.assertEquals("", includeFeedback, this.includeFeedback);
      Assert.assertEquals("", useTwoCMPs, this.useTwoCMPs);
      Assert.assertEquals("", includeFeedback, this.includeFeedback);
      Assert.assertEquals("", useTwoCMPs, this.useTwoCMPs);
   }
}
