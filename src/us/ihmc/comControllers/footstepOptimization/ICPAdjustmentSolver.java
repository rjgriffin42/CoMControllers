package us.ihmc.comControllers.footstepOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;

public class ICPAdjustmentSolver
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int maxNumberOfFootstepsToConsider;

   protected int numberOfFootstepsToConsider;
   protected int totalFreeVariables;
   protected int totalFootstepVariables;
   protected int totalLagrangeMultipliers;

   protected final DenseMatrix64F solverInput_G;
   protected final DenseMatrix64F solverInput_g;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;
   protected final DenseMatrix64F residualCost = new DenseMatrix64F(1, 1);

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F tmpTrans_Aeq;

   private final DenseMatrix64F tmpFootstepTask_H;
   private final DenseMatrix64F tmpFootstepTask_h;

   protected final DenseMatrix64F tmpDynamics_Aeq;
   protected final DenseMatrix64F tmpDynamics_beq;

   protected final DenseMatrix64F solution;
   private final DenseMatrix64F freeVariableSolution;
   private final DenseMatrix64F footstepSolutions;
   private final DenseMatrix64F feedbackSolution;
   private final DenseMatrix64F lagrangeMultiplierSolutions;

   private final DenseMatrix64F costToGo;
   private final DenseMatrix64F tmpCost;

   private final DenseMatrix64F cmpFeedback;

   private final DenseMatrix64F targetICP;
   private final DenseMatrix64F perfectCMP;
   private final DenseMatrix64F finalICPRecursion;

   private double feedbackDynamicEffect;

   protected boolean useFeedback;
   protected boolean useStepAdjustment;
   protected boolean useTwoCMPs;

   private boolean hasPerfectCMP = false;
   private boolean hasFootstepRecursionMutliplier = false;
   private boolean hasReferenceFootstep = false;
   private boolean hasTargetTouchdownICP = false;
   private boolean hasFinalICPRecursion = false;
   private boolean hasFootstepWeight = false;
   private boolean hasFeedbackWeight = false;
   private boolean hasFeedbackDynamicEffect = false;

   private final DenseMatrix64F referenceFootstepVector;
   private final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> cmpFootstepRecursions = new ArrayList<>();

   private final ArrayList<DenseMatrix64F> stepWeights = new ArrayList<>();
   private final DenseMatrix64F feedbackWeight;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final ArrayList<FramePoint2d> footstepSolutionLocations = new ArrayList<>();

   private final double minimumFootstepWeight;
   private final double minimumFeedbackWeight;

   public ICPAdjustmentSolver(ICPAdjustmentControllerParameters icpControllerParameters)
   {
      maxNumberOfFootstepsToConsider = icpControllerParameters.getMaximumNumberOfStepsToConsider();
      minimumFootstepWeight = icpControllerParameters.minimumFootstepWeight();
      minimumFeedbackWeight = icpControllerParameters.minimumFeedbackWeight();

      int maxNumberOfFreeVariables = 3 * maxNumberOfFootstepsToConsider + 2;
      int maxNumberOfLagrangeMultipliers = maxNumberOfFootstepsToConsider + 1;

      solverInput_H = new DenseMatrix64F(maxNumberOfFreeVariables, maxNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maxNumberOfFreeVariables, 1);

      tmpFootstepTask_H = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 3 * maxNumberOfFootstepsToConsider);
      tmpFootstepTask_h = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 1);

      tmpDynamics_Aeq = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, maxNumberOfFootstepsToConsider + 2);
      tmpDynamics_beq = new DenseMatrix64F(2, 1);

      solverInput_Aeq = new DenseMatrix64F(maxNumberOfFreeVariables + 3 * maxNumberOfFootstepsToConsider, 1);
      solverInput_beq = new DenseMatrix64F(maxNumberOfFootstepsToConsider + 2, 1);
      tmpTrans_Aeq = new DenseMatrix64F(1, maxNumberOfFreeVariables + 3 * maxNumberOfFootstepsToConsider);

      solverInput_G = new DenseMatrix64F(maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers, maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers);
      solverInput_g = new DenseMatrix64F(1 + maxNumberOfLagrangeMultipliers, 1);

      solution = new DenseMatrix64F(maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers, 1);
      freeVariableSolution = new DenseMatrix64F(maxNumberOfFreeVariables, 1);
      footstepSolutions = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 1);
      feedbackSolution = new DenseMatrix64F(2, 1);
      lagrangeMultiplierSolutions = new DenseMatrix64F(maxNumberOfLagrangeMultipliers, 1);

      targetICP = new DenseMatrix64F(2, 1);
      perfectCMP = new DenseMatrix64F(2, 1);
      finalICPRecursion = new DenseMatrix64F(2, 1);

      costToGo = new DenseMatrix64F(1, 1);
      tmpCost = new DenseMatrix64F(maxNumberOfFreeVariables, 1);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i ++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));
         cmpFootstepRecursions.add(new DenseMatrix64F(2, 2));
         stepWeights.add(new DenseMatrix64F(2, 2));

         footstepSolutionLocations.add(new FramePoint2d());
      }
      referenceFootstepVector = new DenseMatrix64F(2 * maxNumberOfFootstepsToConsider, 1);

      feedbackWeight = new DenseMatrix64F(2, 2);

      cmpFeedback = new DenseMatrix64F(2, 1);
   }

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);
   public void reset()
   {
      solverInput_H.zero();
      solverInput_h.zero();
      residualCost.zero();

      tmpFootstepTask_H.zero();
      tmpFootstepTask_h.zero();

      solverInput_G.zero();
      solverInput_g.zero();

      solverInput_Aeq.zero();
      solverInput_beq.zero();

      tmpDynamics_Aeq.zero();
      tmpDynamics_beq.zero();

      solution.zero();
      footstepSolutions.zero();
      feedbackSolution.zero();
      lagrangeMultiplierSolutions.zero();

      costToGo.zero();
      cmpFeedback.zero();

      solverInput_H.reshape(totalFreeVariables, totalFreeVariables);
      solverInput_h.reshape(totalFreeVariables, 1);

      tmpFootstepTask_H.reshape(totalFootstepVariables, totalFootstepVariables);
      tmpFootstepTask_h.reshape(totalFootstepVariables, 1);

      solverInput_G.reshape(totalFreeVariables + totalLagrangeMultipliers, totalFreeVariables + totalLagrangeMultipliers);
      solverInput_g.reshape(totalFreeVariables + totalLagrangeMultipliers, 1);

      tmpDynamics_Aeq.reshape(totalFreeVariables, 2);
      tmpDynamics_beq.reshape(2, 1);

      solverInput_Aeq.reshape(totalFreeVariables, totalLagrangeMultipliers);
      solverInput_beq.reshape(totalLagrangeMultipliers, 1);
      tmpTrans_Aeq.reshape(totalLagrangeMultipliers, totalFreeVariables);

      solution.reshape(totalFreeVariables + totalLagrangeMultipliers, 1);
      freeVariableSolution.reshape(totalFreeVariables, 1);
      footstepSolutions.reshape(totalFootstepVariables, 1);
      lagrangeMultiplierSolutions.reshape(totalLagrangeMultipliers, 1);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         cmpFootstepRecursions.get(i).zero();

         stepWeights.get(i).zero();
         MatrixTools.setMatrixBlock(stepWeights.get(i), 0, 0, identity, 0, 0, 2, 2, minimumFootstepWeight);
      }
      feedbackWeight.zero();
      MatrixTools.setMatrixBlock(feedbackWeight, 0, 0, identity, 0, 0, 2, 2, minimumFeedbackWeight);
      tmpCost.reshape(totalFreeVariables, 1);
      referenceFootstepVector.reshape(totalFreeVariables, 1);

      targetICP.zero();
      perfectCMP.zero();
      finalICPRecursion.zero();

      hasPerfectCMP = false;
      hasFootstepRecursionMutliplier = false;
      hasReferenceFootstep = false;
      hasTargetTouchdownICP = false;
      hasFinalICPRecursion = false;
      hasFootstepWeight = false;
      hasFeedbackWeight = false;
      hasFeedbackDynamicEffect = false;
   }

   public void setProblemConditions(int numberOfFootstepsToConsider, boolean useFeedback, boolean useStepAdjustment, boolean useTwoCMPs)
   {
      this.numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider);
      this.useStepAdjustment = useStepAdjustment;
      this.useFeedback = useFeedback;
      this.useTwoCMPs = useTwoCMPs;

      if (!useStepAdjustment)
         this.numberOfFootstepsToConsider = 0;

      if (!useFeedback && !useStepAdjustment)
         throw new RuntimeException("Currently not allowing a single stabilization mechanism.");

      totalLagrangeMultipliers = 2;

      if (useStepAdjustment)
         totalFootstepVariables = 2 * this.numberOfFootstepsToConsider;
      else
         totalFootstepVariables = 0;

      if (useFeedback)
         totalFreeVariables = totalFootstepVariables + 2;
      else
         totalFreeVariables = totalFootstepVariables;
   }

   public void setReferenceFootstepLocation(int footstepIndex, FramePoint2d footstepLocation)
   {
      referenceFootstepLocations.get(footstepIndex).set(0, footstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, footstepLocation.getY());

      referenceFootstepVector.set(2 * footstepIndex, 0, footstepLocation.getX());
      referenceFootstepVector.set(2 * footstepIndex + 1, 0, footstepLocation.getY());

      hasReferenceFootstep = true;
   }

   public void setFootstepRecursionMultipliers(int footstepIndex, double recursion)
   {
      CommonOps.setIdentity(cmpFootstepRecursions.get(footstepIndex));
      CommonOps.scale(recursion, cmpFootstepRecursions.get(footstepIndex));

      hasFootstepRecursionMutliplier = true;
   }

   public void setFootstepWeight(int footstepIndex, double weight)
   {
      CommonOps.setIdentity(stepWeights.get(footstepIndex));
      weight = Math.max(weight, minimumFootstepWeight);
      CommonOps.scale(weight, stepWeights.get(footstepIndex));

      hasFootstepWeight = true;
   }

   public void setFeedbackWeight(double weight)
   {
      CommonOps.setIdentity(feedbackWeight);
      weight = Math.max(weight, minimumFeedbackWeight);
      CommonOps.scale(weight, feedbackWeight);

      hasFeedbackWeight = true;
   }

   public void setFeedbackDynamicEffect(double feedbackDynamicEffect)
   {
      this.feedbackDynamicEffect = feedbackDynamicEffect;

      hasFeedbackDynamicEffect = true;
   }

   public void setFinalICPRecursion(FramePoint2d finalICPRecursion)
   {
      finalICPRecursion.changeFrame(worldFrame);
      this.finalICPRecursion.set(0, 0, finalICPRecursion.getX());
      this.finalICPRecursion.set(1, 0, finalICPRecursion.getY());

      hasFinalICPRecursion = true;
   }

   public void setTargetTouchdownICP(FramePoint2d targetTouchdownICP)
   {
      targetTouchdownICP.changeFrame(worldFrame);
      this.targetICP.set(0, 0, targetTouchdownICP.getX());
      this.targetICP.set(1, 0, targetTouchdownICP.getY());

      hasTargetTouchdownICP = true;
   }

   public void setPerfectCMP(FramePoint2d perfectCMP)
   {
      perfectCMP.changeFrame(worldFrame);
      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());

      hasPerfectCMP = true;
   }

   public void computeMatrices()
   {
      if (!hasPerfectCMP || !hasTargetTouchdownICP || !hasFinalICPRecursion)
         throw new RuntimeException("Have not provided all required inputs to solve the problem.");

      if (useStepAdjustment && (!hasFootstepRecursionMutliplier || !hasReferenceFootstep || !hasFootstepWeight))
         throw new RuntimeException("Need footstep information to solve this problem.");

      computeFootstepCostMatrices();
      MatrixTools.setMatrixBlock(solverInput_H, 0, 0, tmpFootstepTask_H, 0, 0, totalFootstepVariables, totalFootstepVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_h, 0, 0, tmpFootstepTask_h, 0, 0, totalFootstepVariables, 1, 1.0);

      if (useFeedback)
      {
         if (!hasFeedbackWeight || !hasFeedbackDynamicEffect)
            throw new RuntimeException("Have not set up the appropriate feedback weight.");

         MatrixTools.setMatrixBlock(solverInput_H, totalFootstepVariables, totalFootstepVariables, feedbackWeight, 0, 0, 2, 2, 1.0);
      }

      computeConstraintMatrices();

      CommonOps.transpose(solverInput_Aeq, tmpTrans_Aeq);

      // assemble quadratic cost with equalities
      MatrixTools.setMatrixBlock(solverInput_G, 0, 0, solverInput_H, 0, 0, totalFreeVariables, totalFreeVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_G, 0, totalFreeVariables, solverInput_Aeq, 0, 0, totalFreeVariables, totalLagrangeMultipliers, 1.0);
      MatrixTools.setMatrixBlock(solverInput_G, totalFreeVariables, 0, tmpTrans_Aeq, 0, 0, totalLagrangeMultipliers, totalFreeVariables, 1.0);

      // assemble negative linear cost with equalities
      MatrixTools.setMatrixBlock(solverInput_g, 0, 0, solverInput_h, 0, 0, totalFreeVariables, 1, 1.0);
      MatrixTools.setMatrixBlock(solverInput_g, totalFreeVariables, 0, solverInput_beq, 0, 0, totalLagrangeMultipliers, 1, 1.0);
   }

   private final DenseMatrix64F tmpLinearCost = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F tmpPartCost = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F tmpResidualCost = new DenseMatrix64F(1, 1);
   private void computeFootstepCostMatrices()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         // quadratic cost term
         MatrixTools.setMatrixBlock(tmpFootstepTask_H, 2 * i, 2 * i, stepWeights.get(i), 0, 0, 2, 2, 1.0);

         // linear cost term
         tmpLinearCost.zero();
         CommonOps.multTransA(stepWeights.get(i), referenceFootstepLocations.get(i), tmpLinearCost);

         MatrixTools.setMatrixBlock(tmpFootstepTask_h, 2 * i, 0, tmpLinearCost, 0, 0, 2, 1, 1.0);

         // residual cost
         CommonOps.mult(stepWeights.get(i), referenceFootstepLocations.get(i), tmpPartCost);
         CommonOps.multTransA(referenceFootstepLocations.get(i), tmpPartCost, tmpResidualCost);
         CommonOps.add(residualCost, tmpResidualCost, residualCost);
      }
   }

   private void computeConstraintMatrices()
   {
      computeDynamicsConstraintMatrices();

      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, 0, tmpDynamics_Aeq, 0, 0, totalFreeVariables, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, 0, 0, tmpDynamics_beq, 0, 0, 2, 1, 1.0);
   }

   private final DenseMatrix64F ones = CommonOps.identity(2, 2);
   private void computeDynamicsConstraintMatrices()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         MatrixTools.setMatrixBlock(tmpDynamics_Aeq, 2 * i, 0, cmpFootstepRecursions.get(i), 0, 0, 2, 2, 1.0);

      if (useFeedback)
         MatrixTools.setMatrixBlock(tmpDynamics_Aeq, totalFootstepVariables, 0, ones, 0, 0, 2, 2, -feedbackDynamicEffect);

      MatrixTools.setMatrixBlock(tmpDynamics_beq, 0, 0, targetICP, 0, 0, 2, 1, 1.0);
      CommonOps.subtract(tmpDynamics_beq, finalICPRecursion, tmpDynamics_beq);

      CommonOps.subtract(targetICP, finalICPRecursion, tmpDynamics_beq);
   }

   public void solve()
   {
      solver.setA(solverInput_G);
      solver.solve(solverInput_g, solution);

      extractSolutions();

      computeCostToGo();
   }

   private void extractSolutions()
   {
      if (MatrixTools.containsNaN(solution))
         throw new RuntimeException("The solution contains NaN. This case should be made smarter in case.");

      MatrixTools.setMatrixBlock(freeVariableSolution, 0, 0, solution, 0, 0, totalFreeVariables, 1, 1.0);

      if (useFeedback)
      {
         MatrixTools.setMatrixBlock(feedbackSolution, 0, 0, freeVariableSolution, totalFootstepVariables, 0, 2, 1, 1.0);
      }
      else
      {
         MatrixTools.setToZero(feedbackSolution);
      }

      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutions, 0, 0, solution, totalFreeVariables, 1, totalLagrangeMultipliers, 1, 1.0);

      footstepSolutions.set(freeVariableSolution);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         footstepSolutionLocations.get(i).setX(footstepSolutions.get(2 * i));
         footstepSolutionLocations.get(i).setY(footstepSolutions.get(2 * i + 1));
      }

      cmpFeedback.set(perfectCMP);
      CommonOps.add(cmpFeedback, feedbackSolution, cmpFeedback);
   }

   private final DenseMatrix64F tmpCostScalar = new DenseMatrix64F(1, 1);
   private void computeCostToGo()
   {
      CommonOps.mult(solverInput_H, freeVariableSolution, tmpCost);
      CommonOps.multTransA(freeVariableSolution, tmpCost, costToGo);

      CommonOps.multTransA(solverInput_h, freeVariableSolution, tmpCostScalar);
      CommonOps.scale(-2.0, tmpCostScalar);
      CommonOps.add(costToGo, tmpCostScalar, costToGo);

      CommonOps.add(residualCost, costToGo, costToGo);
   }

   public void getFootstepSolutionLocation(int footstepIndex, FramePoint2d footstepLocationToPack)
   {
      footstepLocationToPack.setToZero(worldFrame);
      footstepLocationToPack.set(footstepSolutionLocations.get(footstepIndex));
   }

   public void getCMPFeedbackDifference(FrameVector2d cmpFeedbackDifference)
   {
      cmpFeedbackDifference.setToZero(worldFrame);
      cmpFeedbackDifference.setX(feedbackSolution.get(0, 0));
      cmpFeedbackDifference.setY(feedbackSolution.get(1, 0));
   }

   public void getCMPFeedback(FramePoint2d cmpFeedback)
   {
      cmpFeedback.setToZero(worldFrame);
      cmpFeedback.setX(this.cmpFeedback.get(0, 0));
      cmpFeedback.setY(this.cmpFeedback.get(1, 0));
   }

   public double getCostToGo()
   {
      return costToGo.get(0, 0);
   }
}
