package us.ihmc.comControllers.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class ICPOptimizationSolver
{
   protected final DenseMatrix64F solverInput_G;
   protected final DenseMatrix64F solverInput_g;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_AeqTrans;
   protected final DenseMatrix64F solverInput_beq;

   protected final DenseMatrix64F dynamics_Aeq;
   protected final DenseMatrix64F dynamics_beq;

   protected final int maximumNumberOfFootstepsToConsider;

   protected int numberOfFootstepsToConsider;
   protected int numberOfFreeVariables = 0;
   protected int numberOfFootstepVariables = 0;
   protected int numberOfLagrangeMultipliers = 2;

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters)
   {
      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + 2;
      int maximumNumberOfLagrangeMultipliers = 2;

      int size = maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers;
      solverInput_G = new DenseMatrix64F(size, size);
      solverInput_g = new DenseMatrix64F(size, 1);

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);

      solverInput_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers);
      solverInput_AeqTrans = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, maximumNumberOfFreeVariables);
      solverInput_beq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);

      dynamics_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 2);
      dynamics_beq = new DenseMatrix64F(2, 1);
   }

   public void submitProblemConditions(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback)
   {
      if (!useFeedback && !useStepAdjustment)
      {
         throw new RuntimeException("No possible feedback mechanism available.");
      }

      if (useFeedback && !useStepAdjustment)
         this.numberOfFootstepsToConsider = 0;
      else
         this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      numberOfFootstepVariables = 2 * this.numberOfFootstepsToConsider;
      numberOfFreeVariables = numberOfFootstepsToConsider + 2;
   }

   public void reset()
   {
      int size = numberOfFreeVariables + numberOfLagrangeMultipliers;
      solverInput_G.zero();
      solverInput_g.zero();
      solverInput_G.reshape(size, size);
      solverInput_g.reshape(size, 1);

      solverInput_H.zero();
      solverInput_h.zero();
      solverInput_H.reshape(numberOfFreeVariables, numberOfFreeVariables);
      solverInput_h.reshape(numberOfFreeVariables, 1);

      solverInput_Aeq.zero();
      solverInput_AeqTrans.zero();
      solverInput_beq.zero();
      solverInput_Aeq.reshape(numberOfFreeVariables, numberOfLagrangeMultipliers);
      solverInput_AeqTrans.reshape(numberOfLagrangeMultipliers, numberOfFreeVariables);
      solverInput_beq.reshape(numberOfLagrangeMultipliers, 1);

      dynamics_Aeq.zero();
      dynamics_beq.zero();
      solverInput_Aeq.reshape(numberOfFreeVariables, 2);
      solverInput_AeqTrans.reshape(2, numberOfFreeVariables);
      solverInput_beq.reshape(2, 1);
   }

   public compute()
   {
      assembleTotalProblem();
   }

   private void assembleTotalProblem()
   {
      MatrixTools.setMatrixBlock(solverInput_G, 0, 0, solverInput_H, 0, 0, numberOfFreeVariables, numberOfFreeVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_g, 0, 0, solverInput_h, 0, 0, numberOfFreeVariables, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_G, 0, numberOfFreeVariables, solverInput_Aeq, 0, 0, numberOfFreeVariables, numberOfLagrangeMultipliers, 1.0);
      MatrixTools.setMatrixBlock(solverInput_G, numberOfFreeVariables, 0, solverInput_AeqTrans, 0, 0, numberOfLagrangeMultipliers, numberOfFreeVariables, 1.0);
      MatrixTools.setMatrixBlock(solverInput_g, numberOfLagrangeMultipliers, 0, solverInput_beq, 0, 0, numberOfLagrangeMultipliers, 1, 1.0);
   }

   private void solve()
   {
   }
}
