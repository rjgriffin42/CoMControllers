package us.ihmc.comControllers.icpOptimization;

import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ICPOptimizationSolverTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.0001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStanding()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
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

      ICPOptimizationController icpOptimizationController = new ICPOptimizationController(icpOptimizationParameters, omega, registry);
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
   };
}
