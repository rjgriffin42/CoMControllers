package us.ihmc.comControllers.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ReferenceCentroidalMomentumPivotLocationsCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.vecmath.Quat4d;
import java.util.ArrayList;

public class ICPOptimizationController
{
   private static final String namePrefix = "icpOptimizationCalculator";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);
   private final IntegerYoVariable numberOfFootstepsInPlan = new IntegerYoVariable("numberOfFootstepsInPlan", registry);

   private final BooleanYoVariable useTwoCMPsInControl = new BooleanYoVariable("useTwoCMPsInControl", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable useStepAdjustment = new BooleanYoVariable("useStepAdjustment", registry);

   private final BooleanYoVariable scaleFirstStepWeightWithTime = new BooleanYoVariable("scaleFirstStepWeightWithTime", registry);
   private final BooleanYoVariable scaleFeedbackWeightWithGain = new BooleanYoVariable("scaleFeedbackWeightWithGain", registry);

   private final BooleanYoVariable isStanding = new BooleanYoVariable("isStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("isInTransfer", registry);

   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("timeSpentOnExitCMPInPercentOfStepTime", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("doubleSupportSplitFraction", registry);

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);
   private final DoubleYoVariable timeRemainingInState = new DoubleYoVariable("timeRemainingInState", registry);

   private final YoFramePoint2d controllerCurrentICP = new YoFramePoint2d("controllerCurrentICP", worldFrame, registry);
   private final YoFramePoint2d controllerDesiredICP = new YoFramePoint2d("controllerDesiredICP", worldFrame, registry);
   private final YoFrameVector2d controllerDesiredICPVelocity = new YoFrameVector2d("controllerDesiredICPVelocity", worldFrame, registry);
   private final YoFramePoint2d controllerPerfectCMP = new YoFramePoint2d("controllerPerfectCMP", worldFrame, registry);
   private final YoFramePoint2d controllerFeedbackCMP = new YoFramePoint2d("controllerFeedbackCMP", worldFrame, registry);

   private final YoFramePoint2d cmpOffsetRecursionEffect = new YoFramePoint2d("cmpOffsetRecursionEffect", worldFrame, registry);
   private final YoFramePoint2d finalICPRecursion = new YoFramePoint2d("finalICPRecursion", worldFrame, registry);
   private final YoFramePoint2d stanceCMPProjection = new YoFramePoint2d("stanceCMPProjection", worldFrame, registry);

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> entryOffsets = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> exitOffsets = new ArrayList<>();

   private final DoubleYoVariable footstepWeight = new DoubleYoVariable("footstepWeight", registry);
   private final DoubleYoVariable firstStepWeight = new DoubleYoVariable("firstStepWeight", registry);
   private final DoubleYoVariable feedbackWeight = new DoubleYoVariable("feedbackWeight", registry);
   private final DoubleYoVariable scaledFeedbackWeight = new DoubleYoVariable("scaledFeedbackWeight", registry);
   private final DoubleYoVariable feedbackGain = new DoubleYoVariable("feedbackGain", registry);

   private final DoubleYoVariable omega;

   private final ICPOptimizationSolver solver;
   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final CapturePointPlannerParameters icpPlannerParameters;
   private final int maximumNumberOfFootstepsToConsider;

   public ICPOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
         BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, DoubleYoVariable omega, YoVariableRegistry parentRegistry)
   {
      this.icpPlannerParameters = icpPlannerParameters;
      this.omega = omega;

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      solver = new ICPOptimizationSolver(icpOptimizationParameters);
      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            maximumNumberOfFootstepsToConsider, registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      useFeedback.set(icpOptimizationParameters.useFeedback());
      useStepAdjustment.set(icpOptimizationParameters.useStepAdjustment());

      scaleFirstStepWeightWithTime.set(icpOptimizationParameters.scaleFirstStepWeightWithTime());
      scaleFeedbackWeightWithGain.set(icpOptimizationParameters.scaleFeedbackWeightWithGain());

      footstepWeight.set(icpOptimizationParameters.getFootstepWeight());
      feedbackWeight.set(icpOptimizationParameters.getFeedbackWeight());
      feedbackGain.set(icpOptimizationParameters.getFeedbackGain());

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      doubleSupportSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());

      footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(doubleSupportDuration, singleSupportDuration,
            exitCMPDurationInPercentOfStepTime, doubleSupportSplitFraction, omega, maximumNumberOfFootstepsToConsider, registry);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         entryOffsets.add(new YoFrameVector2d("entryOffset" + i, worldFrame, registry));
         exitOffsets.add(new YoFrameVector2d("exitOffset" + i, worldFrame, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void setStepDurations(double doubleSupportDuration, double singleSupportDuration)
   {
      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
   }

   public void clearPlan()
   {
      upcomingFootsteps.clear();
      footstepRecursionMultiplierCalculator.reset();
      referenceCMPsCalculator.clear();
      numberOfFootstepsInPlan.set(0);
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      upcomingFootsteps.add(footstep);
      referenceCMPsCalculator.addUpcomingFootstep(footstep);
      numberOfFootstepsInPlan.increment();
   }

   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInTransfer.set(false);
   }

   public void initializeForTransfer(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(false);
      isInTransfer.set(true);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider.getIntegerValue(), isInTransfer.getBooleanValue(),
            useTwoCMPsInControl.getBooleanValue());
   }

   public void initializeForSingleSupport(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(false);
      isInTransfer.set(false);

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider.getIntegerValue(), isInTransfer.getBooleanValue(),
            useTwoCMPsInControl.getBooleanValue());
   }

   private final FramePoint2d perfectCMP = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();

   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP)
   {
      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      controllerCurrentICP.set(currentICP);
      controllerDesiredICP.set(desiredICP);
      controllerDesiredICPVelocity.set(desiredICPVelocity);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega.getDoubleValue(), perfectCMP);
      controllerPerfectCMP.set(perfectCMP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      scaleFirstStepWeightWithTime();
      scaleFeedbackWeightWithGain();

      if (isStanding.getBooleanValue())
         doControlForStanding();
      else
         doControlForStepping();

      solver.getCMPFeedback(desiredCMP);
      controllerFeedbackCMP.set(desiredCMP);
   }

   private void doControlForStanding()
   {
      doFeedbackOnlyControl();
   }

   private void doControlForStepping()
   {
      if (useFeedback.getBooleanValue() && !useStepAdjustment.getBooleanValue())
      {
         doFeedbackOnlyControl();
         return;
      }

      if (useFeedback.getBooleanValue())
      {
         solver.setFeedbackConditions(scaledFeedbackWeight.getDoubleValue(), feedbackGain.getDoubleValue());
      }

      if (useStepAdjustment.getBooleanValue())
      {
         int numberOfFootstepsToConsider = this.numberOfFootstepsToConsider.getIntegerValue();
         numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, numberOfFootstepsInPlan.getIntegerValue());
         numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maximumNumberOfFootstepsToConsider);

         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
            submitFootstepConditionsToSolver(footstepIndex);
      }

      computeFinalICPRecursion();
      computeStanceCMPProjection();

      if (useTwoCMPsInControl.getBooleanValue())
      {
         computeCMPOffsetRecursionEffect();
         solver.compute(finalICPRecursion.getFrameTuple2d(), cmpOffsetRecursionEffect.getFrameTuple2d(), controllerCurrentICP.getFrameTuple2d(),
               controllerPerfectCMP.getFrameTuple2d(), stanceCMPProjection.getFrameTuple2d(), omega.getDoubleValue(), timeRemainingInState.getDoubleValue());
      }
      else
      {
         solver.compute(finalICPRecursion.getFrameTuple2d(), null, controllerCurrentICP.getFrameTuple2d(), controllerPerfectCMP.getFrameTuple2d(),
               stanceCMPProjection.getFrameTuple2d(), omega.getDoubleValue(), timeRemainingInState.getDoubleValue());
      }
   }

   private final FramePoint2d blankFramePoint = new FramePoint2d(worldFrame);
   private void doFeedbackOnlyControl()
   {
      solver.submitProblemConditions(0, false, true, false);
      solver.setFeedbackConditions(scaledFeedbackWeight.getDoubleValue(), feedbackGain.getDoubleValue());

      solver.compute(controllerDesiredICP.getFrameTuple2d(), null, controllerCurrentICP.getFrameTuple2d(), controllerPerfectCMP.getFrameTuple2d(), blankFramePoint, 0.0, 0.0);
   }

   private void submitFootstepConditionsToSolver(int footstepIndex)
   {
      double footstepWeight;
      if (footstepIndex == 0)
         footstepWeight = firstStepWeight.getDoubleValue();
      else
         footstepWeight = this.footstepWeight.getDoubleValue();

      double footstepRecursionMultiplier;
      if (useTwoCMPsInControl.getBooleanValue())
      {
         double entryMutliplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(footstepIndex, useTwoCMPsInControl.getBooleanValue());
         double exitMutliplier = footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(footstepIndex, useTwoCMPsInControl.getBooleanValue());

         footstepRecursionMultiplier = entryMutliplier + exitMutliplier;
      }
      else
      {
         footstepRecursionMultiplier = footstepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(footstepIndex, useTwoCMPsInControl.getBooleanValue());
      }

      solver.setFootstepAdjustmentConditions(footstepIndex, footstepRecursionMultiplier, footstepWeight, upcomingFootsteps.get(footstepIndex));
   }

   private final FramePoint2d finalEndingICP2d = new FramePoint2d(worldFrame);
   private void computeFinalICPRecursion()
   {
      FramePoint finalEndingICP = referenceCMPsCalculator.getEntryCMPs().get(numberOfFootstepsToConsider.getIntegerValue()).getFrameTuple();
      finalEndingICP.changeFrame(worldFrame);
      finalEndingICP2d.setByProjectionOntoXYPlane(finalEndingICP);

      finalICPRecursion.set(finalEndingICP2d);
      finalICPRecursion.scale(footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());
   }

   private final FramePoint2d stanceEntryCMP2d = new FramePoint2d(worldFrame);
   private final FramePoint2d stanceExitCMP2d = new FramePoint2d(worldFrame);
   private void computeStanceCMPProjection()
   {
      footstepRecursionMultiplierCalculator.computeStanceFootProjectionMultipliers(timeRemainingInState.getDoubleValue(), useTwoCMPsInControl.getBooleanValue(),
            isInTransfer.getBooleanValue());

      if (useTwoCMPsInControl.getBooleanValue())
      {
         FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();
         FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();
         stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
         stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

         stanceEntryCMP2d.scale(footstepRecursionMultiplierCalculator.getTwoCMPStanceProjectionEntryMutliplier(useTwoCMPsInControl.getBooleanValue()));
         stanceExitCMP2d.scale(footstepRecursionMultiplierCalculator.getTwoCMPStanceProjectionExitMutliplier(useTwoCMPsInControl.getBooleanValue()));

         stanceCMPProjection.set(stanceEntryCMP2d);
         stanceCMPProjection.add(stanceExitCMP2d);
      }
      else
      {
         FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();
         stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

         stanceExitCMP2d.scale(footstepRecursionMultiplierCalculator.getOneCMPStanceProjectionMultiplier(useTwoCMPsInControl.getBooleanValue()));

         stanceCMPProjection.set(stanceExitCMP2d);
      }
   }

   private final FramePoint2d totalOffsetEffect = new FramePoint2d();
   private void computeCMPOffsetRecursionEffect()
   {
      computeTwoCMPOffsets();

      totalOffsetEffect.setToZero();
      cmpOffsetRecursionEffect.setToZero();

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         totalOffsetEffect.set(exitOffsets.get(i).getFrameTuple2d());
         totalOffsetEffect.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPsInControl.getBooleanValue()));

         cmpOffsetRecursionEffect.add(totalOffsetEffect);

         totalOffsetEffect.set(entryOffsets.get(i).getFrameTuple2d());
         totalOffsetEffect.scale(footstepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPsInControl.getBooleanValue()));

         cmpOffsetRecursionEffect.add(totalOffsetEffect);
      }
   }

   private final FramePoint footstepLocation = new FramePoint();
   private final Quat4d footstepOrientation = new Quat4d();
   private final RigidBodyTransform transform = new RigidBodyTransform();

   private void computeTwoCMPOffsets()
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         Footstep upcomingFootstep = upcomingFootsteps.get(i);
         RobotSide stepSide = upcomingFootstep.getRobotSide();
         upcomingFootstep.getPositionIncludingFrame(footstepLocation);
         upcomingFootstep.getOrientationInWorldFrame(footstepOrientation);

         transform.zeroTranslation();
         transform.setRotation(footstepOrientation);

         footstepLocation.changeFrame(worldFrame);

         FrameVector2d entryOffset = entryOffsets.get(i).getFrameTuple2d();
         FrameVector2d exitOffset = exitOffsets.get(i).getFrameTuple2d();

         entryOffset.setToZero(worldFrame);
         exitOffset.setToZero(worldFrame);

         entryOffset.setY(stepSide.negateIfLeftSide(icpPlannerParameters.getEntryCMPInsideOffset()));
         entryOffset.setX(icpPlannerParameters.getEntryCMPForwardOffset());
         exitOffset.setY(stepSide.negateIfLeftSide(icpPlannerParameters.getExitCMPInsideOffset()));
         exitOffset.setX(icpPlannerParameters.getExitCMPForwardOffset());

         entryOffset.applyTransform(transform);
         exitOffset.applyTransform(transform);
      }
   }

   private void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue());
   }


   private void computeTimeRemainingInState()
   {
      if (isStanding.getBooleanValue())
      {
         timeRemainingInState.set(0.0);
      }
      else
      {
         double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
         double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
         double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

         if (isInTransfer.getBooleanValue())
         {
            double remainingTime = totalTimeSpentOnExitCMP - timeInCurrentState.getDoubleValue();
            timeRemainingInState.set(remainingTime);
         }
         else
         {
            double remainingTime = totalTimeSpentOnEntryCMP - timeInCurrentState.getDoubleValue();
            timeRemainingInState.set(remainingTime);
         }
      }
   }

   private void scaleFirstStepWeightWithTime()
   {
      if (scaleFirstStepWeightWithTime.getBooleanValue())
      {
         double alpha = timeRemainingInState.getDoubleValue() / singleSupportDuration.getDoubleValue();
         firstStepWeight.set(footstepWeight.getDoubleValue() / alpha);
      }
      else
      {
         firstStepWeight.set(footstepWeight.getDoubleValue());
      }
   }

   private void scaleFeedbackWeightWithGain()
   {
      if (scaleFeedbackWeightWithGain.getBooleanValue())
      {
         double alpha = Math.pow(feedbackGain.getDoubleValue(), 2);
         scaledFeedbackWeight.set(feedbackWeight.getDoubleValue() / alpha);
      }
      else
      {
         scaledFeedbackWeight.set(feedbackWeight.getDoubleValue());
      }
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controllerFeedbackCMP.getFrameTuple2d(desiredCMPToPack);
   }
}
