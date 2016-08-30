package us.ihmc.comControllers.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ICPOptimizationSolverTest extends ICPOptimizationSolver
{
   private static final YoVariableRegistry rootRegistry = new YoVariableRegistry("robert");

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.0001;

   public ICPOptimizationSolverTest()
   {
      super(icpOptimizationParameters, rootRegistry);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDynamicConstraint()
   {
      double omega = 3.0;
      double remainingTime = 0.5;

      double doubleSupportDuration = 0.2;
      double doubleSupportSplitFraction = 0.5;
      double initialDoubleSupportDuration = doubleSupportSplitFraction * doubleSupportDuration;

      int numberOffootstepsToConsider = 0;
      super.submitProblemConditions(numberOffootstepsToConsider, true, true, false);
      super.setFeedbackConditions(2.0, 0.001);

      double finalICPRecursionMultiplier = Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d finalICP = new FramePoint2d(worldFrame, 0.2, 0.115);
      FramePoint2d finalICPRecursion = new FramePoint2d();
      finalICPRecursion.set(finalICP);
      finalICPRecursion.scale(finalICPRecursionMultiplier);

      double cmpProjectionMultiplier = Math.exp(omega * remainingTime) - Math.exp(-omega * initialDoubleSupportDuration);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.0, -0.155);
      FramePoint2d cmpProjection = new FramePoint2d();
      cmpProjection.set(perfectCMP);
      cmpProjection.scale(cmpProjectionMultiplier);

      FramePoint2d currentICP = new FramePoint2d(worldFrame, 0.1, 0.06);

      compute(finalICPRecursion, null, currentICP, perfectCMP, cmpProjection, omega, timeRemaining);

      FramePoint2d rightHandSide = new FramePoint2d();
      rightHandSide.set(currentICP);
      rightHandSide.scale(Math.exp(omega * timeRemaining));
      rightHandSide.sub(cmpProjection);
      rightHandSide.sub(finalICPRecursion);

      DenseMatrix64F expectedDynamics_beq = new DenseMatrix64F(2, 1);
      expectedDynamics_beq.set(0, 0, rightHandSide.getX());
      expectedDynamics_beq.set(1, 0, rightHandSide.getY());

      JUnitTools.assertMatrixEquals(expectedDynamics_beq, dynamics_beq, epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStanding()
   {
      DoubleYoVariable omega = new DoubleYoVariable("omega", rootRegistry);
      omega.set(3.0);

      FramePoint2d icpDesired = new FramePoint2d();
      FrameVector2d icpDesiredVelocity = new FrameVector2d();
      FramePoint2d icpActual = new FramePoint2d();
      icpActual.set(0.05, 0.07);

      FramePoint2d perfectCMP = new FramePoint2d();
      CapturePointTools.computeDesiredCentroidalMomentumPivot(icpDesired, icpDesiredVelocity, omega.getDoubleValue(), perfectCMP);

      FramePoint2d cmpDesired = new FramePoint2d();

      FramePoint2d cmpGoal = new FramePoint2d();
      cmpGoal.set(icpActual);
      cmpGoal.sub(icpDesired);
      cmpGoal.scale(icpOptimizationParameters.getFeedbackGain());
      cmpGoal.add(perfectCMP);

      ICPOptimizationController icpOptimizationController = new ICPOptimizationController(icpPlannerParameters, icpOptimizationParameters, null, null, omega, rootRegistry);
      icpOptimizationController.initializeForStanding(0.0);

      icpOptimizationController.compute(0.5, icpDesired, icpDesiredVelocity, icpActual);
      icpOptimizationController.getDesiredCMP(cmpDesired);

      JUnitTools.assertPoint2dEquals("", cmpGoal.getPoint(), cmpDesired.getPoint(), epsilon);
   }

   private static final ICPOptimizationParameters icpOptimizationParameters = new ICPOptimizationParameters()
   {
      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override
      public int numberOfFootstepsToConsider()
      {
         return 1;
      }

      @Override public double getFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFeedbackWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackGain()
      {
         return 2.0;
      }

      @Override public boolean scaleFirstStepWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean useFeedback()
      {
         return true;
      }

      @Override public boolean useStepAdjustment()
      {
         return false;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumTimeRemaining()
      {
         return 0.001;
      }
   };

   private static final CapturePointPlannerParameters icpPlannerParameters = new CapturePointPlannerParameters()
   {
      @Override public double getDoubleSupportInitialTransferDuration()
      {
         return 1.0;
      }

      @Override public double getEntryCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getExitCMPInsideOffset()
      {
         return 0;
      }

      @Override public double getEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public boolean useTwoCMPsPerSupport()
      {
         return false;
      }

      @Override public double getMaxEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinEntryCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMaxExitCMPForwardOffset()
      {
         return 0;
      }

      @Override public double getMinExitCMPForwardOffset()
      {
         return 0;
      }
   };
}
