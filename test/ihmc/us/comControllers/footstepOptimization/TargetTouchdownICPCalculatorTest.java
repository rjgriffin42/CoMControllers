package ihmc.us.comControllers.footstepOptimization;

import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class TargetTouchdownICPCalculatorTest
{
   private final static ReferenceFrame worldframe = ReferenceFrame.getWorldFrame();
   private final static double delta = 0.0001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testWithoutFeedback()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);

      omega.set(3.0);

      TargetTouchdownICPCalculator targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega, registry);

      double swingTime = 2.0;
      FramePoint2d perfectCMP = new FramePoint2d(worldframe);
      FramePoint2d currentICP = new FramePoint2d(worldframe);
      FramePoint2d targetICP = new FramePoint2d(worldframe);
      FramePoint2d expectedTargetICP = new FramePoint2d(worldframe);

      currentICP.set(0.2, 0.0);

      targetTouchdownICPCalculator.computeTargetTouchdownICP(swingTime, currentICP, perfectCMP);

      double exponential = Math.exp(omega.getDoubleValue() * swingTime);

      currentICP.scale(exponential);
      perfectCMP.scale(1.0 - exponential);

      expectedTargetICP.set(currentICP);
      expectedTargetICP.add(perfectCMP);

      targetTouchdownICPCalculator.getTargetTouchdownICP(targetICP);

      JUnitTools.assertPoint2dEquals("Target ICP equals", expectedTargetICP.getPoint(), targetICP.getPoint(), delta);
   }
}
