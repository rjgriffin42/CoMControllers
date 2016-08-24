package ihmc.us.comControllers.footstepOptimization;

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

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F tmpTrans_Aeq;

   private final DenseMatrix64F tmpFootstepTask_H;

   protected final DenseMatrix64F tmpDynamics_Aeq;
   protected final DenseMatrix64F tmpDynamics_beq;

   private final DenseMatrix64F tmpTwoCMPProjection_Aeq;
   private final DenseMatrix64F tmpTwoCMPProjection_beq;

   private final DenseMatrix64F footstepSelectionMatrix;

   private final DenseMatrix64F solution;
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

   private double kappa;

   protected boolean includeFeedback;
   protected boolean useTwoCMPs;

   private boolean hasPerfectCMP = false;
   private boolean hasFootstepRecursionMutliplier = false;
   private boolean hasReferenceFootstep = false;
   private boolean hasTargetTouchdownICP = false;
   private boolean hasFinalICPRecursion = false;
   private boolean hasFootstepWeight = false;
   private boolean hasFeedbackWeight = false;
   private boolean hasEffectiveFeedbackGain = false;

   private final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> heelTransforms = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> toeTransforms = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> twoCMPFootstepRecursions = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> oneCMPFootstepRecursions = new ArrayList<>();

   private final ArrayList<DenseMatrix64F> stepWeights = new ArrayList<>();
   private final DenseMatrix64F feedbackWeight;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final ArrayList<FramePoint2d> footstepSolutionLocations = new ArrayList<>();

   public ICPAdjustmentSolver(int maxNumberOfFootstepsToConsider)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      int maxNumberOfFreeVariables = 3 * maxNumberOfFootstepsToConsider + 2;
      int maxNumberOfLagrangeMultipliers = maxNumberOfFootstepsToConsider + 1;

      solverInput_H = new DenseMatrix64F(maxNumberOfFreeVariables, maxNumberOfFreeVariables);

      tmpFootstepTask_H = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 3 * maxNumberOfFootstepsToConsider);

      tmpDynamics_Aeq = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, maxNumberOfFootstepsToConsider + 2);
      tmpDynamics_beq = new DenseMatrix64F(2, 1);

      tmpTwoCMPProjection_Aeq = new DenseMatrix64F(maxNumberOfFreeVariables, maxNumberOfFootstepsToConsider);
      tmpTwoCMPProjection_beq = new DenseMatrix64F(maxNumberOfFootstepsToConsider, 1);

      solverInput_Aeq = new DenseMatrix64F(maxNumberOfFreeVariables + 3 * maxNumberOfFootstepsToConsider, 1);
      solverInput_beq = new DenseMatrix64F(maxNumberOfFootstepsToConsider + 2, 1);
      tmpTrans_Aeq = new DenseMatrix64F(1, maxNumberOfFreeVariables + 3 * maxNumberOfFootstepsToConsider);

      solverInput_G = new DenseMatrix64F(maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers, maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers);
      solverInput_g = new DenseMatrix64F(1 + maxNumberOfLagrangeMultipliers, 1);

      footstepSelectionMatrix = new DenseMatrix64F(2 * maxNumberOfFootstepsToConsider, maxNumberOfFreeVariables);

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

         heelTransforms.add(new DenseMatrix64F(2, 3));
         toeTransforms.add(new DenseMatrix64F(2, 3));
         twoCMPFootstepRecursions.add(new DenseMatrix64F(3, 2)); // transposed when multiplying
         oneCMPFootstepRecursions.add(new DenseMatrix64F(2, 2));

         stepWeights.add(new DenseMatrix64F(2, 2));

         footstepSolutionLocations.add(new FramePoint2d());
      }

      feedbackWeight = new DenseMatrix64F(2, 2);

      cmpFeedback = new DenseMatrix64F(2, 1);
   }

   public void reset()
   {
      solverInput_H.zero();
      tmpFootstepTask_H.zero();

      solverInput_G.zero();
      solverInput_g.zero();

      solverInput_Aeq.zero();
      solverInput_beq.zero();

      tmpDynamics_Aeq.zero();
      tmpDynamics_beq.zero();

      tmpTwoCMPProjection_Aeq.zero();
      tmpTwoCMPProjection_beq.zero();

      solution.zero();
      footstepSolutions.zero();
      feedbackSolution.zero();
      lagrangeMultiplierSolutions.zero();

      footstepSelectionMatrix.zero();

      costToGo.zero();
      cmpFeedback.zero();

      solverInput_H.reshape(totalFreeVariables, totalFreeVariables);
      tmpFootstepTask_H.reshape(totalFootstepVariables, totalFootstepVariables);

      solverInput_G.reshape(totalFreeVariables + totalLagrangeMultipliers, totalFreeVariables + totalLagrangeMultipliers);
      solverInput_g.reshape(totalFreeVariables + totalLagrangeMultipliers, 1);

      tmpDynamics_Aeq.reshape(totalFreeVariables, 2);
      tmpDynamics_beq.reshape(2, 1);

      if (useTwoCMPs)
      {
         tmpTwoCMPProjection_Aeq.reshape(totalFreeVariables, numberOfFootstepsToConsider);
         tmpTwoCMPProjection_beq.reshape(numberOfFootstepsToConsider, 1);

         solverInput_Aeq.reshape(totalFreeVariables, totalLagrangeMultipliers);
         solverInput_beq.reshape(2 + numberOfFootstepsToConsider, 1);
         tmpTrans_Aeq.reshape(totalLagrangeMultipliers, totalFreeVariables);
      }
      else
      {
         tmpTwoCMPProjection_Aeq.reshape(0, totalLagrangeMultipliers);
         tmpTwoCMPProjection_beq.reshape(0, 1);

         solverInput_Aeq.reshape(totalFreeVariables, totalLagrangeMultipliers);
         solverInput_beq.reshape(2, 1);
         tmpTrans_Aeq.reshape(totalLagrangeMultipliers, totalFreeVariables);
      }

      footstepSelectionMatrix.reshape(2 * numberOfFootstepsToConsider, totalFreeVariables);

      tmpTwoCMPProjection_Aeq.reshape(totalFootstepVariables, numberOfFootstepsToConsider);
      tmpTwoCMPProjection_beq.reshape(numberOfFootstepsToConsider, 1);

      solution.reshape(totalFreeVariables + totalLagrangeMultipliers, 1);
      freeVariableSolution.reshape(totalFreeVariables, 1);
      footstepSolutions.reshape(totalFootstepVariables, 1);
      lagrangeMultiplierSolutions.reshape(totalLagrangeMultipliers, 1);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         heelTransforms.get(i).zero();
         toeTransforms.get(i).zero();
         twoCMPFootstepRecursions.get(i).zero();
         oneCMPFootstepRecursions.get(i).zero();

         stepWeights.get(i).zero();
      }
      tmpCost.reshape(totalFreeVariables, 1);

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
      hasEffectiveFeedbackGain = false;
   }

   public void setProblemConditions(int numberOfFootstepsToConsider, boolean includeFeedback, boolean useTwoCMPs)
   {
      this.numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider);
      this.includeFeedback = includeFeedback;
      this.useTwoCMPs = useTwoCMPs;

      if (useTwoCMPs)
      {
         totalFootstepVariables = 3 * this.numberOfFootstepsToConsider;
         totalLagrangeMultipliers = this.numberOfFootstepsToConsider + 2;
      }
      else
      {
         totalFootstepVariables = 2 * this.numberOfFootstepsToConsider;
         totalLagrangeMultipliers = 2;
      }

      if (includeFeedback)
         totalFreeVariables = totalFootstepVariables + 2;
      else
         totalFreeVariables = totalFootstepVariables;
   }

   public void setReferenceFootstepLocation(int footstepIndex, FramePoint2d footstepLocation)
   {
      referenceFootstepLocations.get(footstepIndex).set(0, footstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, footstepLocation.getY());

      hasReferenceFootstep = true;
   }

   public void setReferenceFootstepLocation(int footstepIndex, FramePoint2d footstepLocation, FrameVector2d entryOffset, FrameVector2d exitOffset)
   {
      setReferenceFootstepLocation(footstepIndex, footstepLocation);

      heelTransforms.get(footstepIndex).set(0, 0, 1);
      heelTransforms.get(footstepIndex).set(1, 1, 1);
      heelTransforms.get(footstepIndex).set(0, 2, entryOffset.getX());
      heelTransforms.get(footstepIndex).set(1, 2, entryOffset.getY());

      toeTransforms.get(footstepIndex).set(0, 0, 1);
      toeTransforms.get(footstepIndex).set(1, 1, 1);
      toeTransforms.get(footstepIndex).set(0, 2, exitOffset.getX());
      toeTransforms.get(footstepIndex).set(1, 2, exitOffset.getY());
   }

   public void setFootstepRecursionMultipliers(int footstepIndex, double recursion)
   {
      if (useTwoCMPs)
         throw new RuntimeException("Should be submitting entry and exit recursions multipliers");

      CommonOps.setIdentity(oneCMPFootstepRecursions.get(footstepIndex));
      CommonOps.scale(recursion, oneCMPFootstepRecursions.get(footstepIndex));

      hasFootstepRecursionMutliplier = true;
   }

   public void setFootstepRecursionMultipliers(int footstepIndex, double entryRecursionMultiplier, double exitRecursionMultiplier)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Should be submitting one recursions multiplier");

      for (int row = 0; row < 2; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            double exitValue = exitRecursionMultiplier * toeTransforms.get(footstepIndex).get(row, col);
            double entryValue = entryRecursionMultiplier * heelTransforms.get(footstepIndex).get(row, col);
            twoCMPFootstepRecursions.get(footstepIndex).set(col, row, entryValue + exitValue); // this is transposed
         }
      }

      hasFootstepRecursionMutliplier = true;
   }

   public void setFootstepWeight(int footstepIndex, double weight)
   {
      CommonOps.setIdentity(stepWeights.get(footstepIndex));
      CommonOps.scale(weight, stepWeights.get(footstepIndex));

      hasFootstepWeight = true;
   }

   public void setFeedbackWeight(double weight)
   {
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(weight, feedbackWeight);

      hasFeedbackWeight = true;
   }

   public void setEffectiveFeedbackGain(double effectiveFeedbackGain)
   {
      kappa = effectiveFeedbackGain;

      hasEffectiveFeedbackGain = true;
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
      if (!hasPerfectCMP || !hasFootstepRecursionMutliplier || !hasReferenceFootstep || !hasTargetTouchdownICP || !hasFinalICPRecursion || !hasFootstepWeight)
         throw new RuntimeException("Have not provided all required inputs to solve the problem.");

      computeFootstepCostMatrices();
      MatrixTools.setMatrixBlock(solverInput_H, 0, 0, tmpFootstepTask_H, 0, 0, totalFootstepVariables, totalFootstepVariables, 1.0);

      if (includeFeedback)
      {
         if (!hasFeedbackWeight || !hasEffectiveFeedbackGain)
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
      MatrixTools.setMatrixBlock(solverInput_g, totalFreeVariables, 0, solverInput_beq, 0, 0, totalLagrangeMultipliers, 1, 1.0);
   }

   private void computeFootstepCostMatrices()
   {
      int indexMultiplier;
      if (useTwoCMPs)
         indexMultiplier = 3;
      else
         indexMultiplier = 2;

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         MatrixTools.setMatrixBlock(tmpFootstepTask_H, i * indexMultiplier, i * indexMultiplier, stepWeights.get(i), 0, 0, 2, 2, 1.0);
   }

   private void computeConstraintMatrices()
   {
      computeDynamicsConstraintMatrices();

      if (useTwoCMPs)
      {
         computeTwoCMPProjectionConstraintMatrices();

         MatrixTools.setMatrixBlock(solverInput_Aeq, 0, 1, tmpTwoCMPProjection_Aeq, 0, 0, totalFootstepVariables, numberOfFootstepsToConsider, 1.0);
         MatrixTools.setMatrixBlock(solverInput_beq, 2, 0, tmpTwoCMPProjection_beq, 0, 0, numberOfFootstepsToConsider, 1, 1.0);
      }

      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, 0, tmpDynamics_Aeq, 0, 0, totalFreeVariables, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, 0, 0, tmpDynamics_beq, 0, 0, 2, 1, 1.0);
   }

   private final DenseMatrix64F ones = new DenseMatrix64F(2, 1);
   private void computeDynamicsConstraintMatrices()
   {
      ones.set(0, 0, 1.0);
      ones.set(1, 0, 1.0);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         if (useTwoCMPs)
            MatrixTools.setMatrixBlock(tmpDynamics_Aeq, 3 * i, 0, twoCMPFootstepRecursions.get(i), 0, 0, 3, 2, 1.0);
         else
            MatrixTools.setMatrixBlock(tmpDynamics_Aeq, 2 * i, 0, oneCMPFootstepRecursions.get(i), 0, 0, 2, 2, 1.0);
      }

      if (includeFeedback)
         MatrixTools.setMatrixBlock(tmpDynamics_Aeq, totalFootstepVariables, 0, ones, 0, 0, 2, 1, -kappa);

      MatrixTools.setMatrixBlock(tmpDynamics_beq, 0, 0, targetICP, 0, 0, 2, 1, 1.0);
      CommonOps.subtract(tmpDynamics_beq, finalICPRecursion, tmpDynamics_beq);

      CommonOps.subtract(targetICP, finalICPRecursion, tmpDynamics_beq);
   }

   private void computeTwoCMPProjectionConstraintMatrices()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpTwoCMPProjection_Aeq.set(3 * i, i, 1.0);
         tmpTwoCMPProjection_beq.set(i, 0, 1.0);
      }
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
      computeFootstepSelectionMatrices();

      MatrixTools.setMatrixBlock(freeVariableSolution, 0, 0, solution, 0, 0, totalFreeVariables, 1, 1.0);

      if (includeFeedback)
      {
         MatrixTools.setMatrixBlock(feedbackSolution, 0, 0, freeVariableSolution, totalFootstepVariables, 0, 2, 1, 1.0);
      }

      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutions, 0, 0, solution, totalFreeVariables, 1, totalLagrangeMultipliers, 1, 1.0);

      if (useTwoCMPs)
         CommonOps.mult(footstepSelectionMatrix, freeVariableSolution, footstepSolutions);
      else
         footstepSolutions.set(freeVariableSolution);

      cmpFeedback.set(perfectCMP);
      CommonOps.add(cmpFeedback, feedbackSolution, cmpFeedback);
   }

   private void computeFootstepSelectionMatrices()
   {
      footstepSelectionMatrix.zero();

      if (useTwoCMPs)
      {
         footstepSelectionMatrix.reshape(2 * numberOfFootstepsToConsider, 3 * numberOfFootstepsToConsider);

         for (int i = 0; i < numberOfFootstepsToConsider; i++)
         {
            footstepSelectionMatrix.set(2 * i, 3 * i, 1.0);
            footstepSelectionMatrix.set(2 * i + 1, 3 * i + 1, 1.0);
         }
      }
      else
      {
         footstepSelectionMatrix.reshape(2 * numberOfFootstepsToConsider, 2 * numberOfFootstepsToConsider);
         CommonOps.setIdentity(footstepSelectionMatrix);
      }
   }

   private void computeCostToGo()
   {
      CommonOps.mult(solverInput_H, freeVariableSolution, tmpCost);
      CommonOps.multTransA(freeVariableSolution, tmpCost, costToGo);
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
