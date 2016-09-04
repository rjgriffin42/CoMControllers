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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoMatrix yoWeightG;
   private final YoMatrix yoWeightg;
   private final YoMatrix footstepH;
   private final YoMatrix footsteph;
   private final YoMatrix footstepReferenceLocation;

   protected final DenseMatrix64F solverInput_G;
   protected final DenseMatrix64F solverInput_g;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;
   protected final DenseMatrix64F solverInputResidualCost;

   protected final DenseMatrix64F footstepCost_H;
   protected final DenseMatrix64F footstepCost_h;
   protected final DenseMatrix64F footstepResidualCost;

   protected final DenseMatrix64F footstepRegularizationCost_H;
   protected final DenseMatrix64F footstepRegularizationCost_h;
   protected final DenseMatrix64F footstepRegularizationResidualCost;

   protected final DenseMatrix64F feedbackCost_H;
   protected final DenseMatrix64F feedbackCost_h;
   protected final DenseMatrix64F feedbackResidualCost;

   protected final DenseMatrix64F feedbackRegularizationCost_H;
   protected final DenseMatrix64F feedbackRegularizationCost_h;
   protected final DenseMatrix64F feedbackRegularizationResidualCost;

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_AeqTrans;
   protected final DenseMatrix64F solverInput_beq;

   protected final DenseMatrix64F dynamics_Aeq;
   protected final DenseMatrix64F dynamics_beq;

   protected final ArrayList<DenseMatrix64F> footstepRecursionMutlipliers = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> previousFootstepLocations = new ArrayList<>();
   private final DenseMatrix64F footstepObjectiveVector;

   protected final DenseMatrix64F finalICPRecursion = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F cmpOffsetRecursionEffect = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F stanceCMPProjection = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F currentICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F referenceICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F perfectCMP = new DenseMatrix64F(2, 1);
   protected double currentStateProjection;

   protected final ArrayList<DenseMatrix64F> footstepWeights = new ArrayList<>();
   protected final DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackRegularizationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   protected final DenseMatrix64F solution;
   protected final DenseMatrix64F freeVariableSolution;
   protected final DenseMatrix64F lagrangeMultiplierSolution;
   protected final DenseMatrix64F footstepLocationSolution;
   protected final DenseMatrix64F feedbackDeltaSolution;
   protected final DenseMatrix64F feedbackLocation;
   protected final DenseMatrix64F previousFeedbackDeltaSolution;

   private final DenseMatrix64F tmpCost;
   private final DenseMatrix64F tmpFootstepCost;
   private final DenseMatrix64F tmpFeedbackCost;
   private final DenseMatrix64F costToGo;
   private final DenseMatrix64F footstepCostToGo;
   private final DenseMatrix64F footstepRegularizationCostToGo;
   private final DenseMatrix64F feedbackCostToGo;
   private final DenseMatrix64F feedbackRegularizationCostToGo;

   protected final int maximumNumberOfFootstepsToConsider;

   protected int numberOfFootstepsToConsider;
   protected int numberOfFreeVariables = 0;
   protected int numberOfFootstepVariables = 0;
   protected int numberOfLagrangeMultipliers = 2;

   private boolean useFeedback = false;
   private boolean useStepAdjustment = true;
   private boolean useTwoCMPs = false;

   private boolean hasFootstepRegularizationTerm = false;
   private boolean hasFeedbackRegularizationTerm = false;

   private final double minimumFootstepWeight;
   private final double minimumFeedbackWeight;

   private final double feedbackWeightHardeningMultiplier;

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters)
   {
      this(icpOptimizationParameters, null);
   }

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, YoVariableRegistry parentRegistry)
   {
      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      minimumFootstepWeight = icpOptimizationParameters.getMinimumFootstepWeight();
      minimumFeedbackWeight = icpOptimizationParameters.getMinimumFeedbackWeight();

      feedbackWeightHardeningMultiplier = icpOptimizationParameters.getFeedbackWeightHardeningMultiplier();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + 2;
      int maximumNumberOfLagrangeMultipliers = 2;

      int size = maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers;

      solverInput_G = new DenseMatrix64F(size, size);
      solverInput_g = new DenseMatrix64F(size, 1);

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInputResidualCost = new DenseMatrix64F(1, 1);

      footstepCost_H = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider);
      footstepCost_h = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      footstepResidualCost = new DenseMatrix64F(1, 1);

      footstepRegularizationCost_H = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider);
      footstepRegularizationCost_h = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      footstepRegularizationResidualCost = new DenseMatrix64F(1, 1);

      feedbackCost_H = new DenseMatrix64F(2, 2);
      feedbackCost_h = new DenseMatrix64F(2, 1);
      feedbackResidualCost = new DenseMatrix64F(1, 1);

      feedbackRegularizationCost_H = new DenseMatrix64F(2, 2);
      feedbackRegularizationCost_h = new DenseMatrix64F(2, 1);
      feedbackRegularizationResidualCost = new DenseMatrix64F(1, 1);

      solverInput_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers);
      solverInput_AeqTrans = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, maximumNumberOfFreeVariables);
      solverInput_beq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);

      dynamics_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 2);
      dynamics_beq = new DenseMatrix64F(2, 1);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));
         previousFootstepLocations.add(new DenseMatrix64F(2, 1));

         footstepRecursionMutlipliers.add(new DenseMatrix64F(2, 2));
         footstepWeights.add(new DenseMatrix64F(2, 2));
      }
      footstepObjectiveVector =  new DenseMatrix64F(2 * maximumNumberOfFreeVariables, 1);

      solution = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      lagrangeMultiplierSolution = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);
      freeVariableSolution = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      footstepLocationSolution = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      feedbackDeltaSolution = new DenseMatrix64F(2, 1);
      feedbackLocation = new DenseMatrix64F(2, 1);
      previousFeedbackDeltaSolution = new DenseMatrix64F(2, 1);

      tmpCost = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      tmpFootstepCost = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      tmpFeedbackCost = new DenseMatrix64F(2, 1);
      costToGo = new DenseMatrix64F(1, 1);
      footstepCostToGo = new DenseMatrix64F(1, 1);
      footstepRegularizationCostToGo = new DenseMatrix64F(1, 1);
      feedbackCostToGo = new DenseMatrix64F(1, 1);
      feedbackRegularizationCostToGo = new DenseMatrix64F(1, 1);

      if (parentRegistry != null)
      {
         yoWeightG = new YoMatrix("solverQuadraticCost", maximumNumberOfFreeVariables, maximumNumberOfFreeVariables, registry);
         yoWeightg = new YoMatrix("solverLinearCost", maximumNumberOfFreeVariables, 1, registry);

         footstepH = new YoMatrix("footstepQuadraticCost", 2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider, registry);
         footsteph = new YoMatrix("footstepLinearCost", 2 * maximumNumberOfFootstepsToConsider, 1, registry);

         footstepReferenceLocation = new YoMatrix("footstepReferenceLocation", 2 * maximumNumberOfFootstepsToConsider, 1, registry);

         parentRegistry.addChild(registry);
      }
      else
      {
         yoWeightG = null;
         yoWeightg = null;

         footstepH = null;
         footsteph = null;

         footstepReferenceLocation = null;
      }
   }

   public void submitProblemConditions(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      if (!useFeedback && (!useStepAdjustment || numberOfFootstepsToConsider < 1))
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

      if (useFeedback)
         numberOfFreeVariables = numberOfFootstepVariables + 2;
      else
         numberOfFreeVariables = numberOfFootstepVariables;

      reset();
      reshape();
   }

   private void reset()
   {
      solverInput_G.zero();
      solverInput_g.zero();

      solverInput_H.zero();
      solverInput_h.zero();
      solverInputResidualCost.zero();

      footstepCost_H.zero();
      footstepCost_h.zero();
      footstepResidualCost.zero();

      footstepRegularizationCost_H.zero();
      footstepRegularizationCost_h.zero();
      footstepRegularizationResidualCost.zero();

      feedbackCost_H.zero();
      feedbackCost_h.zero();
      feedbackResidualCost.zero();

      feedbackRegularizationCost_H.zero();
      feedbackRegularizationCost_h.zero();
      feedbackRegularizationResidualCost.zero();

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
      referenceICP.zero();
      perfectCMP.zero();

      footstepRegularizationWeight.zero();
      feedbackWeight.zero();
      feedbackGain.zero();

      solution.zero();
      lagrangeMultiplierSolution.zero();
      freeVariableSolution.zero();
      footstepLocationSolution.zero();
      feedbackDeltaSolution.zero();
      feedbackLocation.zero();

      hasFootstepRegularizationTerm = false;
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

      footstepRegularizationCost_H.reshape(numberOfFootstepVariables, numberOfFootstepVariables);
      footstepRegularizationCost_h.reshape(numberOfFootstepVariables, 1);

      solverInput_Aeq.reshape(numberOfFreeVariables, numberOfLagrangeMultipliers);
      solverInput_AeqTrans.reshape(numberOfLagrangeMultipliers, numberOfFreeVariables);
      solverInput_beq.reshape(numberOfLagrangeMultipliers, 1);

      dynamics_Aeq.reshape(numberOfFreeVariables, 2);

      solution.reshape(numberOfFreeVariables + numberOfLagrangeMultipliers, 1);
      freeVariableSolution.reshape(numberOfFreeVariables, 1);
      lagrangeMultiplierSolution.reshape(numberOfLagrangeMultipliers, 1);
      footstepLocationSolution.reshape(numberOfFootstepVariables, 1);
      footstepObjectiveVector.reshape(numberOfFootstepVariables, 1);
   }

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);
   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double weight, FramePoint2d referenceFootstepLocation)
   {
      this.setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, weight, weight, referenceFootstepLocation);
   }

   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double xWeight, double yWeight, FramePoint2d referenceFootstepLocation)
   {
      setFootstepRecursionMutliplier(footstepIndex, recursionMultiplier);
      setFootstepWeight(footstepIndex, xWeight, yWeight);
      setReferenceFootstepLocation(footstepIndex, referenceFootstepLocation);
   }

   private void setFootstepRecursionMutliplier(int footstepIndex, double recursionMultiplier)
   {
      CommonOps.setIdentity(identity);
      MatrixTools.setMatrixBlock(footstepRecursionMutlipliers.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, recursionMultiplier);
   }

   private void setFootstepWeight(int footstepIndex, double xWeight, double yWeight)
   {
      xWeight = Math.max(minimumFootstepWeight, xWeight);
      yWeight = Math.max(minimumFootstepWeight, yWeight);

      identity.zero();
      identity.set(0, 0, xWeight);
      identity.set(1, 1, yWeight);
      MatrixTools.setMatrixBlock(footstepWeights.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, 1.0);
   }

   private void setReferenceFootstepLocation(int footstepIndex, FramePoint2d referenceFootstepLocation)
   {
      referenceFootstepLocation.changeFrame(worldFrame);
      referenceFootstepLocations.get(footstepIndex).set(0, 0, referenceFootstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, 0, referenceFootstepLocation.getY());
   }

   public void setFeedbackConditions(double feedbackWeight, double feedbackGain)
   {
      this.setFeedbackConditions(feedbackWeight, feedbackWeight, feedbackGain, feedbackGain);
   }

   public void setFeedbackConditions(double feedbackXWeight, double feedbackYWeight, double feedbackXGain, double feedbackYGain)
   {
      feedbackXWeight = Math.max(feedbackXWeight, minimumFeedbackWeight);
      feedbackYWeight = Math.max(feedbackYWeight, minimumFeedbackWeight);

      this.feedbackWeight.zero();
      this.feedbackWeight.set(0, 0, feedbackXWeight);
      this.feedbackWeight.set(1, 1, feedbackYWeight);

      this.feedbackGain.zero();
      this.feedbackGain.set(0, 0, feedbackXGain);
      this.feedbackGain.set(1, 1, feedbackYGain);
   }

   public void setFeedbackRegularizationWeight(double regularizationWeight)
   {
      CommonOps.setIdentity(feedbackRegularizationWeight);
      CommonOps.scale(regularizationWeight, feedbackRegularizationWeight);

      hasFeedbackRegularizationTerm = true;
   }

   public void setUseFeedbackWeightHardening()
   {
      double xWeight = feedbackWeight.get(0, 0);
      double yWeight = feedbackWeight.get(1, 1);

      xWeight *= (1.0 + feedbackWeightHardeningMultiplier * Math.abs(previousFeedbackDeltaSolution.get(0, 0)));
      yWeight *= (1.0 + feedbackWeightHardeningMultiplier * Math.abs(previousFeedbackDeltaSolution.get(1, 0)));

      feedbackWeight.set(0, 0, xWeight);
      feedbackWeight.set(1, 1, yWeight);
   }

   public void setFootstepRegularizationWeight(double regularizationWeight)
   {
      CommonOps.setIdentity(footstepRegularizationWeight);
      CommonOps.scale(regularizationWeight, footstepRegularizationWeight);

      hasFootstepRegularizationTerm = true;
   }

   public void resetFootstepRegularization(int footstepIndex, FramePoint2d previousFootstepLocation)
   {
      previousFootstepLocation.changeFrame(worldFrame);
      previousFootstepLocations.get(footstepIndex).set(0, 0, previousFootstepLocation.getX());
      previousFootstepLocations.get(footstepIndex).set(1, 0, previousFootstepLocation.getY());
   }

   public void resetFeedbackRegularization()
   {
      previousFeedbackDeltaSolution.zero();
   }

   public void compute(FramePoint2d finalICPRecursion, FramePoint2d cmpOffsetRecursionEffect, FramePoint2d currentICP, FramePoint2d perfectCMP,
                       FramePoint2d stanceCMPProjection, double currentStateProjection)
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

      this.currentStateProjection = currentStateProjection;

      if (useTwoCMPs)
      {
         this.cmpOffsetRecursionEffect.set(0, 0, cmpOffsetRecursionEffect.getX());
         this.cmpOffsetRecursionEffect.set(1, 0, cmpOffsetRecursionEffect.getY());
      }

      if (useFeedback)
      {
         addFeedbackTask();

         if (hasFeedbackRegularizationTerm)
            addFeedbackRegularizationTask();
      }

      if (useStepAdjustment)
      {
         addStepAdjustmentTask();

         if (hasFootstepRegularizationTerm)
            addFootstepRegularizationTask();
      }

      addDynamicConstraint();

      assembleTotalProblem();

      solve(solution);

      extractLagrangeMultiplierSolution(lagrangeMultiplierSolution);
      extractFreeVariableSolution(freeVariableSolution);
      if (useStepAdjustment)
      {
         extractFootstepSolutions(footstepLocationSolution);
         setPreviousFootstepSolution(footstepLocationSolution);
      }
      if (useFeedback)
      {
         extractFeedbackDeltaSolution(feedbackDeltaSolution);
         setPreviousFeedbackDeltaSolution(feedbackDeltaSolution);
      }

      yoWeightG.set(solverInput_G);
      yoWeightg.set(solverInput_g);

      computeFeedbackLocation();

      computeCostToGo();
   }

   protected void addFeedbackTask()
   {
      MatrixTools.setMatrixBlock(feedbackCost_H, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
      feedbackCost_h.zero();

      MatrixTools.addMatrixBlock(solverInput_H, numberOfFootstepVariables, numberOfFootstepVariables, feedbackCost_H, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, numberOfFootstepVariables, 0, feedbackCost_h, 0, 0, 2, 1, 1.0);
   }

   private final DenseMatrix64F tmpFootstepObjective = new DenseMatrix64F(2, 1);
   protected void addStepAdjustmentTask()
   {
      footstepObjectiveVector.zero();
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.setMatrixBlock(footstepCost_H, 2 * i, 2 * i, footstepWeights.get(i), 0, 0, 2, 2, 1.0);

         tmpFootstepObjective.zero();
         tmpFootstepObjective.set(referenceFootstepLocations.get(i));
         CommonOps.mult(footstepWeights.get(i), tmpFootstepObjective, tmpFootstepObjective);
         CommonOps.multTransA(referenceFootstepLocations.get(i), tmpFootstepObjective, footstepRegularizationResidualCost);

         MatrixTools.setMatrixBlock(footstepCost_h, 2 * i, 0, tmpFootstepObjective, 0, 0, 2, 1, 1.0);
         CommonOps.addEquals(solverInputResidualCost, footstepRegularizationResidualCost);

         MatrixTools.setMatrixBlock(footstepObjectiveVector, 2 * i, 0, referenceFootstepLocations.get(i), 0, 0, 2, 1, 1.0);
      }
      footstepReferenceLocation.set(footstepObjectiveVector);

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, footstepCost_H, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, footstepCost_h, 0, 0, numberOfFootstepVariables, 1, 1.0);

      footstepH.set(footstepCost_H);
      footsteph.set(footstepCost_h);
   }

   private final DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 1);
   protected void addFeedbackRegularizationTask()
   {
      MatrixTools.setMatrixBlock(feedbackRegularizationCost_H, 0, 0, feedbackRegularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(previousFeedbackDeltaSolution);
      CommonOps.mult(feedbackRegularizationWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(previousFeedbackDeltaSolution, tmpObjective, feedbackRegularizationResidualCost);

      MatrixTools.setMatrixBlock(feedbackRegularizationCost_h, 0, 0, tmpObjective, 0, 0, 2, 1, 1.0);
      CommonOps.addEquals(solverInputResidualCost, feedbackRegularizationResidualCost);

      MatrixTools.addMatrixBlock(solverInput_H, numberOfFootstepVariables, numberOfFootstepVariables, feedbackRegularizationCost_H, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, numberOfFootstepVariables, 1, feedbackRegularizationCost_h, 0, 0, 2, 1, 1.0);
   }

   protected void addFootstepRegularizationTask()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.setMatrixBlock(footstepRegularizationCost_H, 2 * i, 2 * i, footstepRegularizationWeight, 0, 0, 2, 2, 1.0);

         tmpObjective.zero();
         tmpObjective.set(previousFootstepLocations.get(i));
         CommonOps.mult(footstepRegularizationWeight, tmpObjective, tmpObjective);
         CommonOps.multTransA(previousFootstepLocations.get(i), tmpObjective, footstepRegularizationResidualCost);

         MatrixTools.setMatrixBlock(footstepRegularizationCost_h, 2 * i, 0, tmpObjective, 0, 0, 2, 1, 1.0);
         CommonOps.addEquals(solverInputResidualCost, footstepRegularizationResidualCost);
      }

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, footstepRegularizationCost_H, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, footstepRegularizationCost_h, 0, 0, numberOfFootstepVariables, 1, 1.0);
   }

   private void addDynamicConstraint()
   {
      computeDynamicConstraint();

      MatrixTools.addMatrixBlock(solverInput_Aeq, 0, 0, dynamics_Aeq, 0, 0, numberOfFreeVariables, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_beq, 0, 0, dynamics_beq, 0, 0, 2, 1, 1.0);
   }

   private void computeDynamicConstraint()
   {
      if (useFeedback)
         addFeedbackToDynamicConstraint(currentStateProjection);
      if (useStepAdjustment)
         addFootstepRecursionsToDynamicConstraint();

      CommonOps.scale(currentStateProjection, currentICP);

      CommonOps.subtractEquals(currentICP, finalICPRecursion);
      CommonOps.subtractEquals(currentICP, stanceCMPProjection);

      if (useTwoCMPs)
         CommonOps.subtractEquals(currentICP, cmpOffsetRecursionEffect);

      MatrixTools.setMatrixBlock(dynamics_beq, 0, 0, currentICP, 0, 0, 2, 1, 1.0);
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
      MatrixTools.setMatrixBlock(solverInput_g, numberOfFreeVariables, 0, solverInput_beq, 0, 0, numberOfLagrangeMultipliers, 1, 1.0);
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
      MatrixTools.setMatrixBlock(feedbackSolutionToPack, 0, 0, solution, numberOfFootstepVariables, 0, 2, 1, 1.0);
   }

   private void extractLagrangeMultiplierSolution(DenseMatrix64F lagrangeMultiplierSolutionToPack)
   {
      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutionToPack, 0, 0, solution, numberOfFreeVariables, 0, numberOfLagrangeMultipliers, 1, 1.0);
   }

   private void extractFreeVariableSolution(DenseMatrix64F freeVariableSolution)
   {
      MatrixTools.setMatrixBlock(freeVariableSolution, 0, 0, solution, 0, 0, numberOfFreeVariables, 1, 1.0);
   }

   private void setPreviousFootstepSolution(DenseMatrix64F footstepLocationSolution)
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         MatrixTools.setMatrixBlock(previousFootstepLocations.get(i), 0, 0, footstepLocationSolution, 2 * i, 0, 2, 1, 1.0);
   }

   private void setPreviousFeedbackDeltaSolution(DenseMatrix64F feedbackDeltaSolution)
   {
      MatrixTools.setMatrixBlock(previousFeedbackDeltaSolution, 0, 0, feedbackDeltaSolution, 0, 0, 2, 1, 1.0);
   }

   private void computeFeedbackLocation()
   {
      feedbackLocation.set(perfectCMP);
      CommonOps.addEquals(feedbackLocation, feedbackDeltaSolution);
   }

   private final DenseMatrix64F tmpCostScalar = new DenseMatrix64F(1, 1);
   private void computeCostToGo()
   {
      costToGo.zero();
      footstepCostToGo.zero();
      footstepRegularizationCostToGo.zero();
      feedbackCostToGo.zero();
      feedbackRegularizationCostToGo.zero();

      tmpCost.zero();
      tmpFootstepCost.zero();
      tmpFeedbackCost.zero();

      tmpCost.reshape(numberOfFreeVariables, 1);
      tmpFootstepCost.reshape(numberOfFootstepVariables, 1);
      tmpFeedbackCost.reshape(2, 1);

      // quadratic cost;
      CommonOps.mult(solverInput_H, freeVariableSolution, tmpCost);
      CommonOps.multTransA(freeVariableSolution, tmpCost, costToGo);

      CommonOps.mult(footstepCost_H, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepCostToGo);

      CommonOps.mult(footstepRegularizationCost_H, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepRegularizationCostToGo);

      CommonOps.mult(feedbackCost_H, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackCostToGo);

      CommonOps.mult(feedbackRegularizationCost_H, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackRegularizationCostToGo);

      // linear cost
      CommonOps.multTransA(solverInput_h, freeVariableSolution, tmpCostScalar);
      CommonOps.addEquals(costToGo, tmpCostScalar);

      CommonOps.multTransA(footstepCost_h, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepCostToGo, tmpCostScalar);

      CommonOps.multTransA(footstepRegularizationCost_h, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepRegularizationCostToGo, tmpCostScalar);

      CommonOps.multTransA(feedbackCost_h, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackCostToGo, tmpCostScalar);

      CommonOps.multTransA(feedbackRegularizationCost_h, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackRegularizationCostToGo, tmpCostScalar);

      // residual cost
      CommonOps.addEquals(costToGo, solverInputResidualCost);
      CommonOps.addEquals(footstepCostToGo, footstepResidualCost);
      CommonOps.addEquals(footstepRegularizationCostToGo, footstepRegularizationResidualCost);
      CommonOps.addEquals(feedbackCostToGo, feedbackResidualCost);
      CommonOps.addEquals(feedbackResidualCost, feedbackRegularizationResidualCost);
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
      return costToGo.get(0, 0);
   }

   public double getFootstepCostToGo()
   {
      return footstepCostToGo.get(0, 0);
   }

   public double getFootstepRegularizationCostToGo()
   {
      return footstepRegularizationCostToGo.get(0, 0);
   }

   public double getFeedbackCostToGo()
   {
      return feedbackCostToGo.get(0, 0);
   }

   public double getFeedbackRegularizationCostToGo()
   {
      return feedbackRegularizationCostToGo.get(0, 0);
   }
}
