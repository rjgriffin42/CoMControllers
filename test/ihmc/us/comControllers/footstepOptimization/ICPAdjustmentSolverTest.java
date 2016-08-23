package ihmc.us.comControllers.footstepOptimization;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

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
      DoubleYoVariable yoTime = new DoubleYoVariable("t", registry);
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

      double stepLength = 0.2;
      double stanceWidth = 0.1;

      perfectCMP.set(0.0, -stanceWidth); // right foot stance
      currentICP.set(0.1, 0.0);
      desiredFootstep1.set(stepLength, stanceWidth);
      desiredFootstep2.set(2 * stepLength, -stanceWidth);

      double

      double singleSupportDuration = 2.0;
      double doubleSupportDuration = 1.0;
      double remainingTime = 1.0;

      super.setProblemConditions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);

      super.reset();

      checkDimensions(numberOfFootstepsToConsider, includeFeedback, useTwoCMPs);
   }

   private void checkDimensions(int nummberOFFootstepsToConsider, boolean includeFeedback, boolean useTwoCMPs)
   {
      int totalLagrangeMultipliers, totalFreeVariables, totalFootstepVariables;
      if (useTwoCMPs)
      {
         totalFootstepVariables = 3 * nummberOFFootstepsToConsider;
         totalLagrangeMultipliers = 1 + nummberOFFootstepsToConsider;
      }
      else
      {
         totalFootstepVariables = 2 * nummberOFFootstepsToConsider;
         totalLagrangeMultipliers = 1;
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
