package ihmc.us.comControllers.footstepOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.ArrayList;
import java.util.Random;

public class ICPAdjustmentSolverTest extends ICPAdjustmentSolver
{
   private static final double epsilon = 0.00001;
   private static final int maxNumberOfFootstepsToConsider = 5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public ICPAdjustmentSolverTest()
   {
      super(maxNumberOfFootstepsToConsider);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDimensionality()
   {
      int numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider - 2;
      boolean includeFeedback = false;
      boolean useTwoCMPs = false;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      includeFeedback = true;
      useTwoCMPs = false;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      includeFeedback = false;
      useTwoCMPs = true;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      includeFeedback = true;
      useTwoCMPs = true;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);


      numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider + 2;
      includeFeedback = false;
      useTwoCMPs = false;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(maxNumberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(maxNumberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      includeFeedback = false;
      useTwoCMPs = true;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(maxNumberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      includeFeedback = true;
      useTwoCMPs = true;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      checkDimensions(maxNumberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
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

   private void runStepTestNoFeedbackOneCMP(int numberOfFootstepsToConsider)
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      boolean includeFeedback = false;
      boolean useTwoCMPs = false;

      TargetTouchdownICPCalculator targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega, registry);
      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfFootstepsToConsider, registry);

      FramePoint2d perfectCMP = new FramePoint2d(worldFrame);
      FramePoint2d currentICP = new FramePoint2d(worldFrame);

      ArrayList<FramePoint2d> desiredFootsteps = new ArrayList<>();
      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.add(new FramePoint2d(worldFrame));

      FramePoint2d targetTouchdownICP = new FramePoint2d(worldFrame);
      FramePoint2d desiredFinalICP = new FramePoint2d(worldFrame);
      FramePoint2d finalICPRecursion = new FramePoint2d(worldFrame);

      double footstepWeight = 3.0;

      double stepLength = 0.2;
      double stanceWidth = 0.1;

      perfectCMP.set(0.0, -stanceWidth); // right foot stance
      currentICP.set(0.1, 0.0);
      RobotSide stanceSide = RobotSide.RIGHT;

      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
      {
         desiredFootsteps.get(i).set((i + 1) * stepLength, stanceSide.negateIfRightSide(stanceWidth));
         stanceSide = stanceSide.getOppositeSide();
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));

      double singleSupportDuration = 2.0;
      double doubleSupportDuration = 1.0;
      double remainingTime = 1.0;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);
      stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, numberOfFootstepsToConsider, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      submitInformation(stepRecursionMultiplierCalculator, desiredFootsteps, null, null, perfectCMP, targetTouchdownICP, finalICPRecursion, footstepWeight,
                        numberOfFootstepsToConsider, useTwoCMPs);

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatricesNoFeedbackOneCMP(stepRecursionMultiplierCalculator, targetTouchdownICP, finalICPRecursion, numberOfFootstepsToConsider, footstepWeight, useTwoCMPs);

      super.solve();

      Assert.assertTrue(super.getCostToGo() > 0.0);
   }

   private void runStepTestNoFeedbackTwoCMPs(int numberOfFootstepsToConsider)
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      boolean includeFeedback = false;
      boolean useTwoCMPs = true;

      TargetTouchdownICPCalculator targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega, registry);
      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfFootstepsToConsider, registry);

      FrameVector2d entryOffset = new FrameVector2d(worldFrame);
      FrameVector2d exitOffset = new FrameVector2d(worldFrame);
      entryOffset.setX(-0.05);
      exitOffset.setX(0.05);

      FramePoint2d perfectCMP = new FramePoint2d(worldFrame);
      FramePoint2d currentICP = new FramePoint2d(worldFrame);

      ArrayList<FramePoint2d> desiredFootsteps = new ArrayList<>();
      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
         desiredFootsteps.add(new FramePoint2d(worldFrame));

      FramePoint2d targetTouchdownICP = new FramePoint2d(worldFrame);
      FramePoint2d desiredFinalICP = new FramePoint2d(worldFrame);
      FramePoint2d finalICPRecursion = new FramePoint2d(worldFrame);
      FramePoint2d twoCMPOffsetEffect = new FramePoint2d(worldFrame);
      FramePoint2d effectiveICPRecursion = new FramePoint2d(worldFrame);

      double footstepWeight = 3.0;

      double stepLength = 0.2;
      double stanceWidth = 0.1;

      perfectCMP.set(0.0, -stanceWidth); // right foot stance
      currentICP.set(0.1, 0.0);
      RobotSide stanceSide = RobotSide.RIGHT;

      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
      {
         desiredFootsteps.get(i).set((i + 1) * stepLength, stanceSide.negateIfRightSide(stanceWidth));
         stanceSide = stanceSide.getOppositeSide();
      }

      desiredFinalICP.set(desiredFootsteps.get(numberOfFootstepsToConsider));

      double singleSupportDuration = 2.0;
      double doubleSupportDuration = 1.0;
      double remainingTime = 1.0;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      double timeSpentOnExitCMPRatio = 0.5;
      double totalTimeSpentOnExitCMP = timeSpentOnExitCMPRatio * steppingDuration;
      double totalTimeSpentOnEntryCMP = (1 - timeSpentOnExitCMPRatio) * steppingDuration;

      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);
      stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, numberOfFootstepsToConsider, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());

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

      effectiveICPRecursion.set(finalICPRecursion);
      effectiveICPRecursion.add(twoCMPOffsetEffect);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      submitInformation(stepRecursionMultiplierCalculator, desiredFootsteps, entryOffset, exitOffset, perfectCMP, targetTouchdownICP, effectiveICPRecursion,
                        footstepWeight, numberOfFootstepsToConsider, useTwoCMPs);

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatricesNoFeedbackTwoCMPs(stepRecursionMultiplierCalculator, targetTouchdownICP, effectiveICPRecursion, numberOfFootstepsToConsider, footstepWeight,
                                     useTwoCMPs);

      super.solve();

      Assert.assertTrue(super.getCostToGo() > 0.0);
   }

   private void submitInformation(StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator, ArrayList<FramePoint2d> desiredFootsteps,
                                  FrameVector2d entryOffset, FrameVector2d exitOffset, FramePoint2d perfectCMP, FramePoint2d targetTouchdownICP,
                                  FramePoint2d finalICPRecursion, double footstepWeight, int numberOfFootstepsToConsider, boolean useTwoCMPs)
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

   private void checkMatricesNoFeedbackOneCMP(StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator, FramePoint2d targetTouchdownICP,
         FramePoint2d finalICPRecursion, int numberOfFootstepsToConsider, double footstepWeight, boolean useTwoCMPs)
   {
      DenseMatrix64F identity = CommonOps.identity(2, 2);
      DenseMatrix64F dynamicsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         CommonOps.setIdentity(identity);
         CommonOps.scale(stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs), identity);
         CommonOps.extract(tmpDynamics_Aeq, 2 * i, 2 * i + 2, 0, 2, dynamicsSubset, 0, 0);

         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, dynamicsSubset, identity, epsilon);
      }

      DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);
      rightHandSide.set(0, 0, targetTouchdownICP.getX() - finalICPRecursion.getX());
      rightHandSide.set(1, 0, targetTouchdownICP.getY() - finalICPRecursion.getY());
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, rightHandSide, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, solverInput_Aeq, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, solverInput_beq, epsilon);

      DenseMatrix64F weights = CommonOps.identity(2, 2);
      CommonOps.scale(footstepWeight, weights);
      int size = 2 * numberOfFootstepsToConsider;
      DenseMatrix64F zeros = new DenseMatrix64F(size, 1);

      DenseMatrix64F optimizationBlock = new DenseMatrix64F(size, size);
      CommonOps.extract(solverInput_G, 0, size, 0, size, optimizationBlock, 0, 0);
      DenseMatrix64F optimizationEquals = new DenseMatrix64F(size, 1);
      CommonOps.extract(solverInput_g, 0, size, 0, 1, optimizationEquals, 0, 0);

      DenseMatrix64F constraintBlock = new DenseMatrix64F(size, 2);
      CommonOps.extract(solverInput_G, 0, size, size, size + 2, constraintBlock, 0, 0);
      DenseMatrix64F constraintEquals = new DenseMatrix64F(2, 1);
      CommonOps.extract(solverInput_g, size, size + 2, 0, 1, constraintEquals, 0, 0);

      DenseMatrix64F weightsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         int start = 2 * i;
         CommonOps.extract(solverInput_H, start, start + 2, start, start + 2, weightsSubset, 0, 0);

         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, weightsSubset, weights, epsilon);
      }
      // check that the cost was inserted into total problem correctly
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, solverInput_H, optimizationBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, zeros, optimizationEquals, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, constraintBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, constraintEquals, epsilon);
   }

   private void checkMatricesNoFeedbackTwoCMPs(StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator, FramePoint2d targetTouchdownICP,
                                               FramePoint2d finalICPRecursion, int numberOfFootstepsToConsider, double footstepWeight, boolean useTwoCMPs)
   {
      DenseMatrix64F identity = CommonOps.identity(2, 2);
      DenseMatrix64F dynamicsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         double recursionMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs) +
            stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);
         CommonOps.setIdentity(identity);
         CommonOps.scale(recursionMultiplier, identity);
         CommonOps.extract(tmpDynamics_Aeq, 2 * i, 2 * i + 2, 0, 2, dynamicsSubset, 0, 0);

         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, dynamicsSubset, identity, epsilon);
      }

      DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);
      rightHandSide.set(0, 0, targetTouchdownICP.getX() - finalICPRecursion.getX());
      rightHandSide.set(1, 0, targetTouchdownICP.getY() - finalICPRecursion.getY());
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, rightHandSide, epsilon);

      int size = 2 * numberOfFootstepsToConsider;
      DenseMatrix64F dynamicsConstraintBlock = new DenseMatrix64F(size, 2);
      DenseMatrix64F dynamicsConstraintEqual = new DenseMatrix64F(2, 1);
      CommonOps.extract(solverInput_Aeq, 0, size, 0, 2, dynamicsConstraintBlock, 0, 0);
      CommonOps.extract(solverInput_beq, 0, 2, 0, 1, dynamicsConstraintEqual, 0, 0);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, dynamicsConstraintBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, dynamicsConstraintEqual, epsilon);

      DenseMatrix64F weights = CommonOps.identity(2, 2);
      CommonOps.scale(footstepWeight, weights);
      DenseMatrix64F zeros = new DenseMatrix64F(size, 1);

      DenseMatrix64F optimizationBlock = new DenseMatrix64F(size, size);
      DenseMatrix64F optimizationEquals = new DenseMatrix64F(size, 1);
      CommonOps.extract(solverInput_G, 0, size, 0, size, optimizationBlock, 0, 0);
      CommonOps.extract(solverInput_g, 0, size, 0, 1, optimizationEquals, 0, 0);

      DenseMatrix64F constraintBlock = new DenseMatrix64F(size, 2);
      DenseMatrix64F constraintEquals = new DenseMatrix64F(2, 1);
      CommonOps.extract(solverInput_G, 0, size, size, size + 2, constraintBlock, 0, 0);
      CommonOps.extract(solverInput_g, size, size + 2, 0, 1, constraintEquals, 0, 0);

      DenseMatrix64F weightsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         int start = 2 * i;
         CommonOps.extract(solverInput_H, start, start + 2, start, start + 2, weightsSubset, 0, 0);

         JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, weightsSubset, weights, epsilon);
      }
      // check that the cost was inserted into total problem correctly
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, solverInput_H, optimizationBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, zeros, optimizationEquals, epsilon);

      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_Aeq, constraintBlock, epsilon);
      JUnitTools.assertMatrixEquals("Number of steps = " + numberOfFootstepsToConsider, tmpDynamics_beq, constraintEquals, epsilon);
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
