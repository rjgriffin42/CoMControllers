package ihmc.us.comControllers.controllers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;

import java.util.ArrayList;

public class ICPAdjustmentMatrixHelper
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final int maxNumberOfFootstepsToConsider;
   private int numberOfFootstepsToConsider;

   private final DenseMatrix64F solverInput_G;
   private final DenseMatrix64F solverInput_g;

   private final DenseMatrix64F solverInput_H;

   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private final DenseMatrix64F tmpTrans_Aeq;

   private final DenseMatrix64F tmpFootstepTask_H;

   private final DenseMatrix64F tmpDynamics_Aeq;
   private final DenseMatrix64F tmpDynamics_beq;

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
   private final DenseMatrix64F currentICP;
   private final DenseMatrix64F perfectCMP;
   private final DenseMatrix64F finalICPRecursion;

   private double exponent;
   private double kappa;

   private boolean includeFeedback;
   private boolean useTwoCMPs;

   private boolean hasFeedbackWeight;
   private boolean hasFeedbackEffectiveGain;
   private boolean hasExponent;
   private boolean hasCurrentICP;
   private boolean hasPerfectCMP;

   private final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> heelTransforms = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> toeTransforms = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> twoCMPFootstepRecursions = new ArrayList<>();
   private final ArrayList<DenseMatrix64F> oneCMPFootstepRecursions = new ArrayList<>();

   private final ArrayList<DenseMatrix64F> stepWeights = new ArrayList<>();
   private final DenseMatrix64F feedbackWeight;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final ArrayList<FramePoint2d> footstepSolutionLocations = new ArrayList<>();

   public ICPAdjustmentMatrixHelper(int maxNumberOfFootstepsToConsider, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = maxNumberOfFootstepsToConsider;

      int maxNumberOfFreeVariables = 3 * maxNumberOfFootstepsToConsider + 2;
      int maxNumberOfLagrangeMultipliers = maxNumberOfFootstepsToConsider + 1;

      solverInput_H = new DenseMatrix64F(maxNumberOfFreeVariables, maxNumberOfFreeVariables);

      tmpFootstepTask_H = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 3 * maxNumberOfFootstepsToConsider);

      tmpDynamics_Aeq = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 1);
      tmpDynamics_beq = new DenseMatrix64F(1, 1);

      tmpTwoCMPProjection_Aeq = new DenseMatrix64F(maxNumberOfFreeVariables, maxNumberOfFootstepsToConsider);
      tmpTwoCMPProjection_beq = new DenseMatrix64F(maxNumberOfFootstepsToConsider, 1);

      solverInput_Aeq = new DenseMatrix64F(maxNumberOfFreeVariables + 3 * maxNumberOfFootstepsToConsider, 1);
      solverInput_beq = new DenseMatrix64F(maxNumberOfFootstepsToConsider + 1, 1);
      tmpTrans_Aeq = new DenseMatrix64F(1, maxNumberOfFreeVariables + 3 * maxNumberOfFootstepsToConsider);

      solverInput_G = new DenseMatrix64F(maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers, maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers);
      solverInput_g = new DenseMatrix64F(1 + maxNumberOfLagrangeMultipliers, 1);

      solution = new DenseMatrix64F(maxNumberOfFreeVariables + maxNumberOfLagrangeMultipliers, 1);
      freeVariableSolution = new DenseMatrix64F(maxNumberOfFreeVariables, 1);
      footstepSolutions = new DenseMatrix64F(3 * maxNumberOfFootstepsToConsider, 1);
      feedbackSolution = new DenseMatrix64F(2, 1);
      lagrangeMultiplierSolutions = new DenseMatrix64F(maxNumberOfLagrangeMultipliers, 1);

      targetICP = new DenseMatrix64F(2, 1);
      currentICP = new DenseMatrix64F(2, 1);
      perfectCMP = new DenseMatrix64F(2, 1);
      finalICPRecursion = new DenseMatrix64F(2, 1);

      costToGo = new DenseMatrix64F(1, 1);
      tmpCost = new DenseMatrix64F(maxNumberOfFreeVariables, 1);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i ++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));

         heelTransforms.add(new DenseMatrix64F(2, 3));
         toeTransforms.add(new DenseMatrix64F(2, 3));
         twoCMPFootstepRecursions.add(new DenseMatrix64F(3, 1));
         oneCMPFootstepRecursions.add(new DenseMatrix64F(2, 1));

         stepWeights.add(new DenseMatrix64F(2, 2));

         footstepSolutionLocations.add(new FramePoint2d());
      }

      feedbackWeight = new DenseMatrix64F(2, 2);

      cmpFeedback = new DenseMatrix64F(2, 1);

      parentRegistry.addChild(registry);
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

      costToGo.zero();
      cmpFeedback.zero();

      int numberOfFootstepFreeVariables, totalFreeVariables;

      if (useTwoCMPs)
         numberOfFootstepFreeVariables = 3 * numberOfFootstepsToConsider;
      else
         numberOfFootstepFreeVariables = 2 * numberOfFootstepsToConsider;

      totalFreeVariables = numberOfFootstepFreeVariables;
      if (includeFeedback)
         totalFreeVariables += 2;

      solverInput_H.reshape(totalFreeVariables, totalFreeVariables);
      tmpFootstepTask_H.reshape(numberOfFootstepFreeVariables, numberOfFootstepFreeVariables);

      int numberOfLagrangeMultipliers = 1;
      if (useTwoCMPs)
         numberOfLagrangeMultipliers += numberOfFootstepsToConsider;

      lagrangeMultiplierSolutions.reshape(numberOfLagrangeMultipliers, 1);

      solverInput_G.reshape(totalFreeVariables + numberOfLagrangeMultipliers, totalFreeVariables, numberOfLagrangeMultipliers);
      solverInput_g.reshape(1 + numberOfLagrangeMultipliers, 1);

      solverInput_Aeq.reshape(totalFreeVariables + numberOfLagrangeMultipliers, 1);
      solverInput_beq.reshape(numberOfLagrangeMultipliers, 1);

      solution.reshape(totalFreeVariables + numberOfLagrangeMultipliers, 1);
      freeVariableSolution.reshape(totalFreeVariables, 1);
      footstepSolutions.reshape(numberOfFootstepFreeVariables, 1);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         heelTransforms.get(i).zero();
         toeTransforms.get(i).zero();
         twoCMPFootstepRecursions.get(i).zero();
         oneCMPFootstepRecursions.get(i).zero();

         stepWeights.get(i).zero();
      }

      targetICP.zero();
      currentICP.zero();
      perfectCMP.zero();
      finalICPRecursion.zero();

      hasFeedbackWeight = false;
      hasExponent = false;
      hasFeedbackEffectiveGain = false;
   }

   public void setProblemConditions(int numberOfFootstepsToConsider, boolean includeFeedback, boolean useTwoCMPs)
   {
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.includeFeedback = includeFeedback;
      this.useTwoCMPs = useTwoCMPs;
   }

   public void setReferenceFootstepLocation(int footstepIndex, FramePoint2d footstepLocation)
   {
      referenceFootstepLocations.get(footstepIndex).set(0, footstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, footstepLocation.getY());
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

      for (int i = 0; i < 2; i++)
         oneCMPFootstepRecursions.get(footstepIndex).set(i, 1, recursion);
   }

   public void setFootstepRecursionMultipliers(int footstepIndex, double entryRecursion, double exitRecursion)
   {
      if (!useTwoCMPs)
         throw new RuntimeException("Should be submitting one recursions multiplier");

      for (int row = 0; row < 2; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            double exitValue = exitRecursion * toeTransforms.get(footstepIndex).get(row, col);
            double entryValue = entryRecursion * heelTransforms.get(footstepIndex).get(row, col);
            twoCMPFootstepRecursions.get(footstepIndex).add(row, col, entryValue + exitValue);
         }
      }
   }

   public void setPerfectCMP(FramePoint2d perfectCMP)
   {
      hasPerfectCMP = true;

      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());
   }

   public void setCurrentICP(FramePoint2d currentICP)
   {
      hasCurrentICP = true;

      this.currentICP.set(0, 0, currentICP.getX());
      this.currentICP.set(1, 0, currentICP.getY());
   }

   public void setFootstepWeight(int footstepIndex, double weight)
   {
      CommonOps.setIdentity(stepWeights.get(footstepIndex));
      CommonOps.scale(weight, stepWeights.get(footstepIndex));
   }

   public void setFeedbackWeight(double weight)
   {
      hasFeedbackWeight = true;

      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(weight, feedbackWeight);
   }

   public void setEffectiveFeedbackGain(double effectiveFeedbackGain)
   {
      hasFeedbackEffectiveGain = true;

      kappa = effectiveFeedbackGain;
   }

   public void setExponentialExponent(double exponent)
   {
      hasExponent = true;
      this.exponent = exponent;
   }

   public void setFinalICPRecursion(FramePoint2d finalICPRecursion)
   {
      finalICPRecursion.changeFrame(worldFrame);
      this.finalICPRecursion.set(0, 0, finalICPRecursion.getX());
      this.finalICPRecursion.set(1, 0, finalICPRecursion.getY());
   }

   public void computeMatrices()
   {
      computeFootstepCostMatrices();

      int indexMultiplier;
      int totalLagrangeMultipliers = 1;
      if (useTwoCMPs)
      {
         indexMultiplier = 3;
         totalLagrangeMultipliers += numberOfFootstepsToConsider;
      }
      else
         indexMultiplier = 2;

      int totalFreeVariables = indexMultiplier * numberOfFootstepsToConsider;

      MatrixTools.setMatrixBlock(solverInput_H, 0, 0, tmpFootstepTask_H, 0, 0, totalFreeVariables, totalFreeVariables, 1.0);

      if (includeFeedback)
      {
         if (!hasFeedbackWeight)
            throw new RuntimeException("Have not set up the feedback weight");

         MatrixTools.setMatrixBlock(solverInput_H, totalFreeVariables, totalFreeVariables, feedbackWeight, 0, 0, 2, 2, 1.0);

         totalFreeVariables += 2;
      }

      computeTargetICPValue();

      computeConstraintMatrices();

      CommonOps.transpose(solverInput_Aeq, tmpTrans_Aeq);

      MatrixTools.setMatrixBlock(solverInput_G, 0, 0, solverInput_H, 0, 0, totalFreeVariables, totalFreeVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_G, 0, totalFreeVariables, solverInput_Aeq, 0, 0, totalFreeVariables, totalLagrangeMultipliers, 1.0);
      MatrixTools.setMatrixBlock(solverInput_G, totalFreeVariables, 0, tmpTrans_Aeq, 0, 0, totalLagrangeMultipliers, totalFreeVariables, 1.0);

      MatrixTools.setMatrixBlock(solverInput_g, 1, 0, solverInput_beq, 0, 0, totalLagrangeMultipliers, 1, 1.0);
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

   private void computeTargetICPValue()
   {
      if (!hasExponent)
         throw new RuntimeException("ICP evolution exponent has not been set.");
      if (!hasCurrentICP)
         throw new RuntimeException("Current ICP has not been set.");
      if (!hasPerfectCMP)
         throw new RuntimeException("Perfect CMP has not been set.");

      if (includeFeedback)
      {
         if (!hasFeedbackEffectiveGain)
            throw new RuntimeException("Effective feedback gain has not been set.");

         targetICP.set(currentICP);
         CommonOps.scale((1 + Math.exp(exponent)), currentICP);
         CommonOps.add(targetICP, -Math.exp(exponent), perfectCMP, targetICP);
      }
      else
      {
         targetICP.set(currentICP);
         CommonOps.scale(Math.exp(exponent), currentICP);
         CommonOps.add(targetICP, (1 - Math.exp(exponent)), perfectCMP, targetICP);
      }
   }

   private void computeConstraintMatrices()
   {
      computeDynamicsConstraintMatrices();

      int numberOfFreeVariables;
      if (useTwoCMPs)
      {
         numberOfFreeVariables = 3 * numberOfFootstepsToConsider;

         computeTwoCMPProjectionConstraintMatrices();

         MatrixTools.setMatrixBlock(solverInput_Aeq, 0, 1, tmpTwoCMPProjection_Aeq, 0, 0, numberOfFreeVariables, numberOfFootstepsToConsider, 1.0);
         MatrixTools.setMatrixBlock(solverInput_beq, 1, 0, tmpTwoCMPProjection_beq, 0, 0, numberOfFootstepsToConsider, 1, 1.0);
      }
      else
      {
         numberOfFreeVariables = 2 * numberOfFootstepsToConsider;
      }

      if (includeFeedback)
         numberOfFreeVariables += 2;


      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, 0, tmpDynamics_Aeq, 0, 0, numberOfFreeVariables, 1, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, 0, 0, tmpDynamics_beq, 0, 0, 1, 1, 1.0);
   }

   private void computeDynamicsConstraintMatrices()
   {
      tmpDynamics_Aeq.zero();
      tmpDynamics_beq.zero();

      int multiplier;
      int numberOfFreeVariables;
      if (useTwoCMPs)
         multiplier = 3;
      else
         multiplier = 2;
      numberOfFreeVariables = multiplier * numberOfFootstepsToConsider;

      if (includeFeedback)
         numberOfFreeVariables += 2;

      tmpDynamics_Aeq.reshape(numberOfFreeVariables, 1);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         if (useTwoCMPs)
            MatrixTools.setMatrixBlock(tmpDynamics_Aeq, multiplier * i, 0, twoCMPFootstepRecursions.get(i), 0, 0, multiplier, 1, 1.0);
         else
            MatrixTools.setMatrixBlock(tmpDynamics_Aeq, multiplier * i, 0, oneCMPFootstepRecursions.get(i), 0, 0, multiplier, 1, 1.0);
      }

      CommonOps.subtract(targetICP, finalICPRecursion, tmpDynamics_beq);
   }

   private void computeTwoCMPProjectionConstraintMatrices()
   {
      tmpTwoCMPProjection_Aeq.zero();
      tmpTwoCMPProjection_beq.zero();

      tmpTwoCMPProjection_Aeq.reshape(3 * numberOfFootstepsToConsider, numberOfFootstepsToConsider);
      tmpTwoCMPProjection_beq.reshape(numberOfFootstepsToConsider, 1);

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         tmpTwoCMPProjection_Aeq.set(3 * i, i, 1.0);
         tmpTwoCMPProjection_beq.set(i, 1, 1.0);
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

      int numberOfFreeVariables, numberOfLagrangeMultipliers;
      if (useTwoCMPs)
      {
         numberOfFreeVariables = 3 * numberOfFootstepsToConsider;
         numberOfLagrangeMultipliers = numberOfFootstepsToConsider;
      }
      else
         numberOfFreeVariables = 2 * numberOfFootstepsToConsider;

      numberOfLagrangeMultipliers += 1;

      MatrixTools.setMatrixBlock(freeVariableSolution, 0, 0, solution, 0, 0, numberOfFreeVariables, 1, 1.0);

      if (includeFeedback)
      {
         MatrixTools.setMatrixBlock(feedbackSolution, 0, 0, solution, numberOfFreeVariables, 0, 2, 1, 1.0);
         numberOfFreeVariables += 2;
      }
      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutions, 0, 0, solution, numberOfFreeVariables, 1, numberOfLagrangeMultipliers, 1, 1.0);

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
      cmpFeedbackDifference.setX(cmpFeedback.get(0, 0));
      cmpFeedbackDifference.setY(cmpFeedback.get(1, 0));
   }

   public double getCostToGo()
   {
      return costToGo.get(0, 0);
   }
}
