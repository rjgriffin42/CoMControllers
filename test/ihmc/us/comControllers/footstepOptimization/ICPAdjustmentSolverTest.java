package ihmc.us.comControllers.footstepOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.ArrayList;

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

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testOneStep()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      int numberOfFootstepsToConsider = 1;
      boolean includeFeedback = false;
      boolean useTwoCMPs = false;

      TargetTouchdownICPCalculator targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega, registry);
      StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega, maxNumberOfFootstepsToConsider, registry);

      FramePoint2d perfectCMP = new FramePoint2d(worldFrame);
      FramePoint2d currentICP = new FramePoint2d(worldFrame);
      FramePoint2d desiredFootstep1 = new FramePoint2d(worldFrame);
      FramePoint2d desiredFootstep2 = new FramePoint2d(worldFrame);

      FramePoint2d targetTouchdownICP = new FramePoint2d(worldFrame);
      FramePoint2d desiredFinalICP = new FramePoint2d(worldFrame);
      FramePoint2d finalICPRecursion = new FramePoint2d(worldFrame);

      double footstepWeight = 3.0;

      double stepLength = 0.2;
      double stanceWidth = 0.1;

      perfectCMP.set(0.0, -stanceWidth); // right foot stance
      currentICP.set(0.1, 0.0);
      desiredFootstep1.set(stepLength, stanceWidth);
      desiredFootstep2.set(2 * stepLength, -stanceWidth);

      desiredFinalICP.set(desiredFootstep2);

      double singleSupportDuration = 2.0;
      double doubleSupportDuration = 1.0;
      double remainingTime = 1.0;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;

      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime, currentICP, perfectCMP);
      stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, numberOfFootstepsToConsider, useTwoCMPs);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);

      double finalICPRecursionMultiplier = stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier();
      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();
      super.setReferenceFootstepLocation(0, desiredFootstep1);
      super.setReferenceFootstepLocation(1, desiredFootstep2);

      super.setPerfectCMP(perfectCMP);
      super.setTargetTouchdownICP(targetTouchdownICP);
      super.setFinalICPRecursion(finalICPRecursion);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         super.setFootstepRecursionMultipliers(i, stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, includeFeedback));
         super.setFootstepWeight(i, footstepWeight);
      }

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatricesNoFeedback(stepRecursionMultiplierCalculator, targetTouchdownICP, finalICPRecursion, numberOfFootstepsToConsider, footstepWeight);

      super.solve();

      Assert.assertTrue(super.getCostToGo() > 0.0);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTwoSteps()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      omega.set(3.0);

      int numberOfFootstepsToConsider = 2;
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

      double finalICPRecursionMultiplier = stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier();
      finalICPRecursion.set(desiredFinalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
      super.reset();

      for (int i = 0; i < numberOfFootstepsToConsider + 1; i++)
         super.setReferenceFootstepLocation(i, desiredFootsteps.get(i));

      super.setPerfectCMP(perfectCMP);
      super.setTargetTouchdownICP(targetTouchdownICP);
      super.setFinalICPRecursion(finalICPRecursion);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         super.setFootstepRecursionMultipliers(i, stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, includeFeedback));
         super.setFootstepWeight(i, footstepWeight);
      }

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.computeMatrices();

      checkMatricesNoFeedback(stepRecursionMultiplierCalculator, targetTouchdownICP, finalICPRecursion, numberOfFootstepsToConsider, footstepWeight);

      super.solve();

      Assert.assertTrue(super.getCostToGo() > 0.0);
   }

   private void checkMatricesNoFeedback(StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator, FramePoint2d targetTouchdownICP,
         FramePoint2d finalICPRecursion, int numberOfFootstepsToConsider, double footstepWeight)
   {
      DenseMatrix64F identity = CommonOps.identity(2, 2);
      DenseMatrix64F dynamicsSubset = new DenseMatrix64F(2, 2);
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         CommonOps.setIdentity(identity);
         CommonOps.scale(stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, includeFeedback), identity);
         CommonOps.extract(tmpDynamics_Aeq, 2 * i, 2 * i + 2, 0, 2, dynamicsSubset, 0, 0);

         JUnitTools.assertMatrixEquals(dynamicsSubset, identity, epsilon);
      }

      DenseMatrix64F rightHandSide = new DenseMatrix64F(2, 1);
      rightHandSide.set(0, 0, targetTouchdownICP.getX() - finalICPRecursion.getX());
      rightHandSide.set(1, 0, targetTouchdownICP.getY() - finalICPRecursion.getY());
      JUnitTools.assertMatrixEquals(tmpDynamics_beq, rightHandSide, epsilon);

      JUnitTools.assertMatrixEquals(tmpDynamics_Aeq, solverInput_Aeq, epsilon);
      JUnitTools.assertMatrixEquals(tmpDynamics_beq, solverInput_beq, epsilon);

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

         JUnitTools.assertMatrixEquals(weightsSubset, weights, epsilon);
      }
      // check that the cost was inserted into total problem correctly
      JUnitTools.assertMatrixEquals(solverInput_H, optimizationBlock, epsilon);
      JUnitTools.assertMatrixEquals(zeros, optimizationEquals, epsilon);

      JUnitTools.assertMatrixEquals(tmpDynamics_Aeq, constraintBlock, epsilon);
      JUnitTools.assertMatrixEquals(tmpDynamics_beq, constraintEquals, epsilon);

   }

   private void checkDimensions(int numberOFFootstepsToConsider, boolean includeFeedback, boolean useTwoCMPs)
   {
      int totalLagrangeMultipliers, totalFreeVariables, totalFootstepVariables;
      if (useTwoCMPs)
      {
         totalFootstepVariables = 3 * numberOFFootstepsToConsider;
         totalLagrangeMultipliers = 2 + numberOFFootstepsToConsider;
      }
      else
      {
         totalFootstepVariables = 2 * numberOFFootstepsToConsider;
         totalLagrangeMultipliers = 2;
      }

      if (includeFeedback)
      {
         totalFreeVariables = totalFootstepVariables + 2;
      }
      else
      {
         totalFreeVariables = totalFootstepVariables;
      }

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
