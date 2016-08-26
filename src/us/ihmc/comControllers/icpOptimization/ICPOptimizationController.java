package us.ihmc.comControllers.icpOptimization;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPOptimizationController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);

   private final BooleanYoVariable useTwoCMPsInControl = new BooleanYoVariable("useTwoCMPsInControl", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable useStepAdjustment = new BooleanYoVariable("useStepAdjustment", registry);

   private final BooleanYoVariable scaleFirstStepWeightWithTime = new BooleanYoVariable("scaleFirstStepWeightWithTime", registry);
   private final BooleanYoVariable scaleFeedbackWeightWithGain = new BooleanYoVariable("scaleFeedbackWeightWithGain", registry);

   private final BooleanYoVariable isStanding = new BooleanYoVariable("isStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("isInTransfer", registry);

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);
   private final DoubleYoVariable timeRemainingInState = new DoubleYoVariable("timeRemainingInState", registry);

   private final YoFramePoint2d controllerCurrentICP = new YoFramePoint2d("controllerCurrentICP", worldFrame, registry);
   private final YoFramePoint2d controllerDesiredICP = new YoFramePoint2d("controllerDesiredICP", worldFrame, registry);
   private final YoFrameVector2d controllerDesiredICPVelocity = new YoFrameVector2d("controllerDesiredICPVelocity", worldFrame, registry);
   private final YoFramePoint2d controllerPerfectCMP = new YoFramePoint2d("controllerPerfectCMP", worldFrame, registry);

   private final DoubleYoVariable footstepWeight = new DoubleYoVariable("footstepWeight", registry);
   private final DoubleYoVariable firstStepWeight = new DoubleYoVariable("firstStepWeight", registry);
   private final DoubleYoVariable feedbackWeight = new DoubleYoVariable("feedbackWeight", registry);
   private final DoubleYoVariable scaledFeedbackWeight = new DoubleYoVariable("scaledFeedbackWeight", registry);
   private final DoubleYoVariable feedbackGain = new DoubleYoVariable("feedbackGain", registry);

   private final DoubleYoVariable omega;

   private final ICPOptimizationSolver solver;

   public ICPOptimizationController(ICPOptimizationParameters icpOptimizationParameters, DoubleYoVariable omega)
   {
      solver = new ICPOptimizationSolver(icpOptimizationParameters);
      this.omega = omega;
   }

   public void initializeStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInTransfer.set(false);
   }

   private final FramePoint2d perfectCMP = new FramePoint2d();

   public void compute(double time, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP)
   {
      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      controllerCurrentICP.set(currentICP);
      controllerDesiredICP.set(desiredICP);
      controllerDesiredICPVelocity.set(desiredICPVelocity);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega.getDoubleValue(), perfectCMP);
      controllerPerfectCMP.set(perfectCMP);

      if (isStanding.getBooleanValue())
      {
         solver.submitProblemConditions(0, false, true, false);
      }
      else
      {
         solver.submitProblemConditions(numberOfFootstepsToConsider.getIntegerValue(), useStepAdjustment.getBooleanValue(), useFeedback.getBooleanValue(),
                                        useTwoCMPsInControl.getBooleanValue());
      }
   }
}
