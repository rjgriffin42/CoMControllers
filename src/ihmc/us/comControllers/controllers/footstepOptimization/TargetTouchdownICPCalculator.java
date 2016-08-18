package ihmc.us.comControllers.controllers.footstepOptimization;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TargetTouchdownICPCalculator
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final  YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d targetTouchdownICP = new YoFramePoint2d("targetTouchdownICP", worldFrame, registry);

   private final DoubleYoVariable omega;

   public TargetTouchdownICPCalculator(DoubleYoVariable omega, YoVariableRegistry parentRegistry)
   {
      this.omega = omega;

      parentRegistry.addChild(registry);
   }

   private final FramePoint2d tmpICP = new FramePoint2d();
   private final FramePoint2d tmpCMP = new FramePoint2d();

   public void computeTargetTouchdownICP(double timeRemaining, FramePoint2d currentICP, FramePoint2d perfectCMP, boolean useFeedback)
   {
      currentICP.changeFrame(worldFrame);
      perfectCMP.changeFrame(worldFrame);

      tmpICP.set(currentICP);
      tmpCMP.set(perfectCMP);

      double exponential = Math.exp(omega.getDoubleValue() * timeRemaining);

      if (useFeedback)
      {
         tmpICP.scale(1 + exponential);
         tmpCMP.scale(-exponential);
      }
      else
      {
         tmpICP.scale(exponential);
         tmpCMP.scale(1 - exponential);
      }

      targetTouchdownICP.set(tmpICP);
      targetTouchdownICP.add(tmpCMP);
   }

   public void getTargetTouchdownICP(FramePoint2d targetTouchdownICPToPack)
   {
      targetTouchdownICP.getFrameTuple2d(targetTouchdownICPToPack);
   }
}
