package us.ihmc.comControllers.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.ArrayList;

public class ICPOptimizationSolver
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final DenseMatrix64F solverInput_G;
   protected final DenseMatrix64F solverInput_g;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;

   protected final DenseMatrix64F footstepCost_H;
   protected final DenseMatrix64F footstepCost_h;

   protected final DenseMatrix64F feedbackCost_H;
   protected final DenseMatrix64F feedbackCost_h;

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_AeqTrans;
   protected final DenseMatrix64F solverInput_beq;

   protected final DenseMatrix64F dynamics_Aeq;
   protected final DenseMatrix64F dynamics_beq;

   protected final ArrayList<DenseMatrix64F> footstepRecursionMutlipliers = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();

   protected final DenseMatrix64F finalICPRecursion = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F cmpOffsetRecursionEffect = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F stanceCMPProjection = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F currentICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F perfectCMP = new DenseMatrix64F(2, 1);
   protected double omega;
   protected double timeRemaining;

   protected final ArrayList<DenseMatrix64F> footstepWeights = new ArrayList<>();
   protected final DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   protected final DenseMatrix64F solution;
   protected final DenseMatrix64F lagrangeMultiplierSolution;
   protected final DenseMatrix64F footstepLocationSolution;
   protected final DenseMatrix64F feedbackDeltaSolution;
   protected final DenseMatrix64F feedbackLocation;

   protected final int maximumNumberOfFootstepsToConsider;

   protected int numberOfFootstepsToConsider;
   protected int numberOfFreeVariables = 0;
   protected int numberOfFootstepVariables = 0;
   protected int numberOfLagrangeMultipliers = 2;

   private boolean useFeedback = false;
   private boolean useStepAdjustment = true;
   private boolean useTwoCMPs = false;

   private final double minimumFootstepWeight;
   private final double minimumFeedbackWeight;

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry parentRegistry)
   {
      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      minimumFootstepWeight = icpOptimizationParameters.getMinimumFootstepWeight();
      minimumFeedbackWeight = icpOptimizationParameters.getMinimumFeedbackWeight();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + 2;
      int maximumNumberOfLagrangeMultipliers = 2;

      int size = maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers;

      solverInput_G = new DenseMatrix64F(size, size);
      solverInput_g = new DenseMatrix64F(size, 1);

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);

      footstepCost_H = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider);
      footstepCost_h = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);

      feedbackCost_H = new DenseMatrix64F(2, 2);
      feedbackCost_h = new DenseMatrix64F(2, 1);

      solverInput_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers);
      solverInput_AeqTrans = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, maximumNumberOfFreeVariables);
      solverInput_beq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);

      dynamics_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 2);
      dynamics_beq = new DenseMatrix64F(2, 1);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));
         footstepRecursionMutlipliers.add(new DenseMatrix64F(2, 2));
         footstepWeights.add(new DenseMatrix64F(2, 2));
      }

      solution = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      lagrangeMultiplierSolution = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);
      footstepLocationSolution = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      feedbackDeltaSolution = new DenseMatrix64F(2, 1);
      feedbackLocation = new DenseMatrix64F(2, 1);
   }

   public void submitProblemConditions(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      if (!useFeedback && !useStepAdjustment)
      {
         throw new RuntimeException("No possible feedback mechanism available.");
      }

      if (this.useFeedback == useFeedback && this.useStepAdjustment == useStepAdjustment && this.useTwoCMPs == useTwoCMPs && this.numberOfFootstepsToConsider == numberOfFootstepsToConsider)
      {
         reset();
         return;
      }

      this.useFeedback = useFeedback;
      this.useStepAdjustment = useStepAdjustment;
      this.useTwoCMPs = useTwoCMPs;

      if (useFeedback && !useStepAdjustment)
         this.numberOfFootstepsToConsider = 0;
      else
         this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      numberOfFootstepVariables = 2 * this.numberOfFootstepsToConsider;
      numberOfFreeVariables = numberOfFootstepVariables + 2;

      reset();
      reshape();
   }

   private void reset()
   {
      solverInput_G.zero();
      solverInput_g.zero();

      solverInput_H.zero();
      solverInput_h.zero();

      footstepCost_H.zero();
      footstepCost_h.zero();

      feedbackCost_H.zero();
      feedbackCost_h.zero();

      solverInput_Aeq.zero();
      solverInput_AeqTrans.zero();
      solverInput_beq.zero();

      dynamics_Aeq.zero();
      dynamics_beq.zero();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         footstepRecursionMutlipliers.get(i).zero();
         footstepWeights.get(i).zero();
      }

      finalICPRecursion.zero();
      cmpOffsetRecursionEffect.zero();
      currentICP.zero();
      perfectCMP.zero();

      feedbackWeight.zero();
      feedbackGain.zero();

      solution.zero();
      lagrangeMultiplierSolution.zero();
      footstepLocationSolution.zero();
      feedbackDeltaSolution.zero();
      feedbackLocation.zero();
   }

   private void reshape()
   {
      int size = numberOfFreeVariables + numberOfLagrangeMultipliers;
      solverInput_G.reshape(size, size);
      solverInput_g.reshape(size, 1);

      solverInput_H.reshape(numberOfFreeVariables, numberOfFreeVariables);
      solverInput_h.reshape(numberOfFreeVariables, 1);

      footstepCost_H.reshape(numberOfFootstepVariables, numberOfFootstepVariables);
      footstepCost_h.reshape(numberOfFootstepVariables, 1);

      solverInput_Aeq.reshape(numberOfFreeVariables, numberOfLagrangeMultipliers);
      solverInput_AeqTrans.reshape(numberOfLagrangeMultipliers, numberOfFreeVariables);
      solverInput_beq.reshape(numberOfLagrangeMultipliers, 1);

      dynamics_Aeq.reshape(numberOfFreeVariables, 2);

      solution.reshape(numberOfFreeVariables + numberOfLagrangeMultipliers, 1);
      lagrangeMultiplierSolution.reshape(numberOfLagrangeMultipliers, 1);
      footstepLocationSolution.reshape(numberOfFootstepVariables, 1);
   }

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);

   private final FramePoint tmpReferenceFootstepLocation = new FramePoint();
   private final FramePoint2d tmpReferenceFootstepLocation2d = new FramePoint2d();

   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double weight, Footstep referenceFootstep)
   {
      referenceFootstep.getPositionIncludingFrame(tmpReferenceFootstepLocation);
      tmpReferenceFootstepLocation.changeFrame(worldFrame);
      tmpReferenceFootstepLocation2d.setByProjectionOntoXYPlane(tmpReferenceFootstepLocation);

      setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, weight, tmpReferenceFootstepLocation2d);
   }

   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double weight, FramePoint2d referenceFootstepLocation)
   {
      setFootstepRecursionMutliplier(footstepIndex, recursionMultiplier);
      setFootstepWeight(footstepIndex, weight);
      setReferenceFootstepLocation(footstepIndex, referenceFootstepLocation);
   }

   private void setFootstepRecursionMutliplier(int footstepIndex, double recursionMultiplier)
   {
      CommonOps.setIdentity(identity);
      MatrixTools.addMatrixBlock(footstepRecursionMutlipliers.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, recursionMultiplier);
   }

   private void setFootstepWeight(int footstepIndex, double weight)
   {
      CommonOps.setIdentity(identity);
      weight = Math.max(minimumFootstepWeight, weight);
      MatrixTools.addMatrixBlock(footstepWeights.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, weight);
   }

   private void setReferenceFootstepLocation(int footstepIndex, FramePoint2d referenceFootstepLocation)
   {
      referenceFootstepLocation.changeFrame(worldFrame);
      referenceFootstepLocations.get(footstepIndex).set(0, 0, referenceFootstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, 0, referenceFootstepLocation.getY());
   }

   public void setFeedbackConditions(double feedbackWeight, double feedbackGain)
   {
      feedbackWeight = Math.max(minimumFeedbackWeight, feedbackWeight);

      CommonOps.setIdentity(this.feedbackWeight);
      CommonOps.setIdentity(this.feedbackGain);

      CommonOps.scale(feedbackWeight, this.feedbackWeight);
      CommonOps.scale(feedbackGain, this.feedbackGain);
   }

   public void compute(FramePoint2d finalICPRecursion, FramePoint2d cmpOffsetRecursionEffect, FramePoint2d currentICP, FramePoint2d perfectCMP,
                       FramePoint2d stanceCMPProjection, double omega, double timeRemaining)
   {
      finalICPRecursion.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);
      perfectCMP.changeFrame(worldFrame);
      stanceCMPProjection.changeFrame(worldFrame);

      if (cmpOffsetRecursionEffect != null)
         cmpOffsetRecursionEffect.changeFrame(worldFrame);

      this.finalICPRecursion.set(0, 0, finalICPRecursion.getX());
      this.finalICPRecursion.set(1, 0, finalICPRecursion.getY());

      this.currentICP.set(0, 0, currentICP.getX());
      this.currentICP.set(1, 0, currentICP.getY());

      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());

      this.stanceCMPProjection.set(0, 0, stanceCMPProjection.getX());
      this.stanceCMPProjection.set(1, 0, stanceCMPProjection.getY());

      this.timeRemaining = timeRemaining;
      this.omega = omega;

      if (useTwoCMPs)
      {
         this.cmpOffsetRecursionEffect.set(0, 0, cmpOffsetRecursionEffect.getX());
         this.cmpOffsetRecursionEffect.set(1, 0, cmpOffsetRecursionEffect.getY());
      }

      if (useFeedback)
         addFeedbackTask();

      if (useStepAdjustment)
         addStepAdjustmentTask();

      addDynamicConstraint();

      assembleTotalProblem();

      solve(solution);

      extractLagrangeMultiplierSolution(lagrangeMultiplierSolution);
      if (useStepAdjustment)
         extractFootstepSolutions(footstepLocationSolution);
      if (useFeedback)
         extractFeedbackDeltaSolution(feedbackDeltaSolution);

      computeFeedbackLocation();

      computeCostToGo();
   }

   private void addFeedbackTask()
   {
      MatrixTools.addMatrixBlock(feedbackCost_H, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
      feedbackCost_h.zero();

      MatrixTools.addMatrixBlock(solverInput_H, numberOfFootstepVariables, numberOfFootstepVariables, feedbackCost_H, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, numberOfFootstepVariables, 0, feedbackCost_h, 0, 0, 2, 1, 1.0);
   }

   private final DenseMatrix64F tmpFootstepObjective = new DenseMatrix64F(2, 1);
   private void addStepAdjustmentTask()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.addMatrixBlock(footstepCost_H, 2 * i, 2 * i, footstepWeights.get(i), 0, 0, 2, 2, 1.0);

         tmpFootstepObjective.zero();
         tmpFootstepObjective.set(referenceFootstepLocations.get(i));
         CommonOps.mult(footstepWeights.get(i), tmpFootstepObjective, tmpFootstepObjective);

         MatrixTools.addMatrixBlock(footstepCost_h, 2 * i, 0, tmpFootstepObjective, 0, 0, 2, 1, 1.0);
      }

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, footstepCost_H, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, footstepCost_h, 0, 0, numberOfFootstepVariables, 1, 1.0);
   }

   private void addDynamicConstraint()
   {
      computeDynamicConstraint();

      MatrixTools.addMatrixBlock(solverInput_Aeq, 0, 0, dynamics_Aeq, 0, 0, numberOfFreeVariables, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_beq, 0, 0, dynamics_beq, 0, 0, 2, 1, 1.0);
   }

   private void computeDynamicConstraint()
   {
      double currentStateProjection = Math.exp(omega * timeRemaining);

      if (useFeedback)
         addFeedbackToDynamicConstraint(currentStateProjection);
      if (useStepAdjustment)
         addFootstepRecursionsToDynamicConstraint();

      CommonOps.scale(currentStateProjection, currentICP);

      CommonOps.subtractEquals(currentICP, finalICPRecursion);
      CommonOps.subtractEquals(currentICP, stanceCMPProjection);

      if (useTwoCMPs)
         CommonOps.subtractEquals(currentICP, cmpOffsetRecursionEffect);

      MatrixTools.addMatrixBlock(dynamics_beq, 0, 0, currentICP, 0, 0, 2, 1, 1.0);
   }

   private void addFeedbackToDynamicConstraint(double currentStateProjection)
   {
      CommonOps.invert(feedbackGain);
      CommonOps.scale(currentStateProjection, feedbackGain);

      MatrixTools.addMatrixBlock(dynamics_Aeq, numberOfFootstepVariables, 0, feedbackGain, 0, 0, 2, 2, 1.0);
   }

   private void addFootstepRecursionsToDynamicConstraint()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.addMatrixBlock(dynamics_Aeq, 2 * i, 0, footstepRecursionMutlipliers.get(i), 0, 0, 2, 2, 1.0);
      }
   }

   private void assembleTotalProblem()
   {
      CommonOps.transpose(solverInput_Aeq, solverInput_AeqTrans);

      // add matrix minimization objective
      MatrixTools.setMatrixBlock(solverInput_G, 0, 0, solverInput_H, 0, 0, numberOfFreeVariables, numberOfFreeVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_g, 0, 0, solverInput_h, 0, 0, numberOfFreeVariables, 1, 1.0);

      // add constraints
      MatrixTools.setMatrixBlock(solverInput_G, 0, numberOfFreeVariables, solverInput_Aeq, 0, 0, numberOfFreeVariables, numberOfLagrangeMultipliers, 1.0);
      MatrixTools.setMatrixBlock(solverInput_G, numberOfFreeVariables, 0, solverInput_AeqTrans, 0, 0, numberOfLagrangeMultipliers, numberOfFreeVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_g, numberOfLagrangeMultipliers, 0, solverInput_beq, 0, 0, numberOfLagrangeMultipliers, 1, 1.0);
   }

   private void solve(DenseMatrix64F solutionToPack)
   {
      solver.setA(solverInput_G);
      solver.solve(solverInput_g, solutionToPack);

      if (MatrixTools.containsNaN(solutionToPack))
      {
         PrintTools.debug("number of steps = " + numberOfFootstepsToConsider);
         PrintTools.debug("solverInput_G = " + solverInput_G);
         PrintTools.debug("solverInput_g = " + solverInput_g);
         throw new RuntimeException("had a NaN");
      }
   }

   private void extractFootstepSolutions(DenseMatrix64F footstepLocationSolutionToPack)
   {
      MatrixTools.setMatrixBlock(footstepLocationSolutionToPack, 0, 0, solution, 0, 0, numberOfFootstepVariables, 1, 1.0);
   }

   private void extractFeedbackDeltaSolution(DenseMatrix64F feedbackSolutionToPack)
   {
      MatrixTools.setMatrixBlock(feedbackSolutionToPack, 0, 0, solution, numberOfFootstepsToConsider, 0, 2, 1, 1.0);
   }

   private void extractLagrangeMultiplierSolution(DenseMatrix64F lagrangeMultiplierSolutionToPack)
   {
      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutionToPack, 0, 0, solution, numberOfFreeVariables, 0, numberOfLagrangeMultipliers, 1, 1.0);
   }

   private void computeFeedbackLocation()
   {
      feedbackLocation.set(perfectCMP);
      CommonOps.addEquals(feedbackLocation, feedbackDeltaSolution);
   }

   private void computeCostToGo()
   {
      //todo
   }

   public void getFootstepSolutionLocation(int footstepIndex, FramePoint2d footstepLocationToPack)
   {
      footstepLocationToPack.setToZero(worldFrame);
      footstepLocationToPack.setX(footstepLocationSolution.get(2 * footstepIndex, 0));
      footstepLocationToPack.setY(footstepLocationSolution.get(2 * footstepIndex + 1, 0));
   }

   public void getCMPFeedbackDifference(FrameVector2d cmpFeedbackDifferenceToPack)
   {
      cmpFeedbackDifferenceToPack.setToZero(worldFrame);
      cmpFeedbackDifferenceToPack.setX(feedbackDeltaSolution.get(0, 0));
      cmpFeedbackDifferenceToPack.setY(feedbackDeltaSolution.get(1, 0));
   }

   public void getCMPFeedback(FramePoint2d cmpFeedbackToPack)
   {
      cmpFeedbackToPack.setToZero(worldFrame);
      cmpFeedbackToPack.setX(feedbackLocation.get(0, 0));
      cmpFeedbackToPack.setY(feedbackLocation.get(1, 0));
   }

   public double getCostToGo()
   {
      return 0.0; // todo
   }
}