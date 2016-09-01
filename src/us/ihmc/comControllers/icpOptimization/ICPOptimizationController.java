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
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
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
   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);

   private final BooleanYoVariable useTwoCMPsInControl = new BooleanYoVariable("useTwoCMPsInControl", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable useStepAdjustment = new BooleanYoVariable("useStepAdjustment", registry);

   private final BooleanYoVariable scaleFirstStepWeightWithTime = new BooleanYoVariable("scaleFirstStepWeightWithTime", registry);
   private final BooleanYoVariable scaleFeedbackWeightWithGain = new BooleanYoVariable("scaleFeedbackWeightWithGain", registry);

   private final BooleanYoVariable isStanding = new BooleanYoVariable(yoNamePrefix + "IsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable(yoNamePrefix + "IsInTransfer", registry);
   private final BooleanYoVariable isInTransferEntry = new BooleanYoVariable(yoNamePrefix + "IsInTransferEntry", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(yoNamePrefix + "IsInitialTransfer", registry);

   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(yoNamePrefix + "DoubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(yoNamePrefix + "SingleSupportDuration", registry);
   private final DoubleYoVariable initialDoubleSupportDuration = new DoubleYoVariable(yoNamePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("timeSpentOnExitCMPInPercentOfStepTime", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("doubleSupportSplitFraction", registry);
   private final BooleanYoVariable stepTimesInitialized = new BooleanYoVariable("stepTimesInitialized", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>("controllerTransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>("controllerSupportSide", registry, RobotSide.class, true);

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);
   private final DoubleYoVariable timeRemainingInState = new DoubleYoVariable("timeRemainingInState", registry);

   private final YoFramePoint2d controllerCurrentICP = new YoFramePoint2d("controllerCurrentICP", worldFrame, registry);
   private final YoFramePoint2d controllerDesiredICP = new YoFramePoint2d("controllerDesiredICP", worldFrame, registry);
   private final YoFrameVector2d controllerDesiredICPVelocity = new YoFrameVector2d("controllerDesiredICPVelocity", worldFrame, registry);
   private final YoFramePoint2d controllerPerfectCMP = new YoFramePoint2d("controllerPerfectCMP", worldFrame, registry);

   private final YoFramePoint2d controllerFeedbackCMP = new YoFramePoint2d("controllerFeedbackCMP", worldFrame, registry);
   private final YoFrameVector2d controllerFeedbackCMPDelta = new YoFrameVector2d("controllerFeedbackCMPDelta", worldFrame, registry);
   private final DoubleYoVariable controllerCostToGo = new DoubleYoVariable("costToGo", registry);

   private final YoFramePoint2d stanceEntryCMP = new YoFramePoint2d("stanceEntryCMP", worldFrame, registry);
   private final YoFramePoint2d stanceExitCMP = new YoFramePoint2d("stanceExitCMP", worldFrame, registry);
   private final YoFramePoint2d previousStanceExitCMP = new YoFramePoint2d("previousStanceExitCMP", worldFrame, registry);
   private final YoFramePoint2d stanceCMPProjection = new YoFramePoint2d("stanceCMPProjection", worldFrame, registry);

   private final YoFramePoint2d finalICP = new YoFramePoint2d("finalICP", worldFrame, registry);
   private final YoFramePoint2d finalICPRecursion = new YoFramePoint2d("finalICPRecursion", worldFrame, registry);
   private final YoFramePoint2d cmpOffsetRecursionEffect = new YoFramePoint2d("cmpOffsetRecursionEffect", worldFrame, registry);

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<FrameVector2d> entryOffsets = new ArrayList<>();
   private final ArrayList<FrameVector2d> exitOffsets = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> yoEntryOffsets = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> yoExitOffsets = new ArrayList<>();
   private final ArrayList<YoFramePoint2d> footstepSolutions = new ArrayList<>();

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
   private final ICPOptimizationParameters icpOptimizationParameters;
   private final int maximumNumberOfFootstepsToConsider;

   public ICPOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
         BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, DoubleYoVariable omega, YoVariableRegistry parentRegistry)
   {
      this.icpPlannerParameters = icpPlannerParameters;
      this.icpOptimizationParameters = icpOptimizationParameters;
      this.omega = omega;

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();
      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      initialDoubleSupportDuration.set(icpPlannerParameters.getDoubleSupportInitialTransferDuration());

      solver = new ICPOptimizationSolver(icpOptimizationParameters, registry);
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

      footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(exitCMPDurationInPercentOfStepTime, doubleSupportSplitFraction, omega,
            maximumNumberOfFootstepsToConsider, registry);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         yoEntryOffsets.add(new YoFrameVector2d("entryOffset" + i, worldFrame, registry));
         yoExitOffsets.add(new YoFrameVector2d("exitOffset" + i, worldFrame, registry));
         footstepSolutions.add(new YoFramePoint2d("footstepSolutionLocation" + i, worldFrame, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void setStepDurations(double doubleSupportDuration, double singleSupportDuration)
   {
      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      stepTimesInitialized.set(true);
   }

   public void clearPlan()
   {
      upcomingFootsteps.clear();
      footstepRecursionMultiplierCalculator.reset();
      referenceCMPsCalculator.clear();
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      upcomingFootsteps.add(footstep);
      referenceCMPsCalculator.addUpcomingFootstep(footstep);
   }

   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInTransfer.set(false);
      isInTransferEntry.set(false);
      isInitialTransfer.set(true);

      footstepRecursionMultiplierCalculator.resetTimes();
   }

   public void initializeForTransfer(double initialTime, RobotSide transferToSide)
   {
      this.initialTime.set(initialTime);
      this.transferToSide.set(transferToSide);
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;
      isInTransfer.set(true);
      isInTransferEntry.set(true);

      // fixme submitting these must be smarter
      footstepRecursionMultiplierCalculator.resetTimes();
      if (isInitialTransfer.getBooleanValue())
         footstepRecursionMultiplierCalculator.submitTimes(0, initialDoubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      else
         footstepRecursionMultiplierCalculator.submitTimes(0, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      for (int i = 1; i < numberOfFootstepsToConsider.getIntegerValue() + 1; i++)
         footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider.getIntegerValue(), isInTransfer.getBooleanValue(),
            useTwoCMPsInControl.getBooleanValue());

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoCMPsInControl.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding.getBooleanValue(), transferToSide);
      referenceCMPsCalculator.update();
   }

   public void initializeForSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.initialTime.set(initialTime);
      this.supportSide.set(supportSide);
      isStanding.set(false);
      isInTransfer.set(false);
      isInTransferEntry.set(false);
      isInitialTransfer.set(false);

      // fixme submitting these must be smarter
      footstepRecursionMultiplierCalculator.resetTimes();
      footstepRecursionMultiplierCalculator.submitTimes(0, 0.0, singleSupportDuration.getDoubleValue());

      for (int i = 1; i < numberOfFootstepsToConsider.getIntegerValue() + 1; i++)
         footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      footstepRecursionMultiplierCalculator.submitTimes(numberOfFootstepsToConsider.getIntegerValue() + 1, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider.getIntegerValue(), isInTransfer.getBooleanValue(),
            useTwoCMPsInControl.getBooleanValue());

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoCMPsInControl.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromSingleSupport(supportSide);
      referenceCMPsCalculator.update();
   }

   private final FramePoint2d perfectCMP = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FrameVector2d desiredCMPDelta = new FrameVector2d();

   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP)
   {
      if (!stepTimesInitialized.getBooleanValue())
         throw new RuntimeException("Step timing has never been initialized!");

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
         doFeedbackOnlyControl();
      else
         doControlForStepping();

      solver.getCMPFeedback(desiredCMP);
      solver.getCMPFeedbackDifference(desiredCMPDelta);

      controllerFeedbackCMP.set(desiredCMP);
      controllerFeedbackCMPDelta.set(desiredCMPDelta);
      controllerCostToGo.set(solver.getCostToGo());
   }

   private void doControlForStanding()
   {
      doFeedbackOnlyControl();
   }

   private final FramePoint2d locationSolution = new FramePoint2d();
   private void doControlForStepping()
   {
      if (useFeedback.getBooleanValue() && !useStepAdjustment.getBooleanValue())
      {
         doFeedbackOnlyControl();
         return;
      }

      int numberOfFootstepsToConsider = this.numberOfFootstepsToConsider.getIntegerValue();
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, upcomingFootsteps.size());
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maximumNumberOfFootstepsToConsider);

      solver.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment.getBooleanValue(), useFeedback.getBooleanValue(),
                                     useTwoCMPsInControl.getBooleanValue());

      if (useFeedback.getBooleanValue())
      {
         solver.setFeedbackConditions(scaledFeedbackWeight.getDoubleValue(), feedbackGain.getDoubleValue());
      }

      if (useStepAdjustment.getBooleanValue())
      {
         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
            submitFootstepConditionsToSolver(footstepIndex);
      }

      computeFinalICPRecursion();
      computeStanceCMPProjection();


      if (useTwoCMPsInControl.getBooleanValue())
      {
         computeCMPOffsetRecursionEffect();
         solver.compute(finalICPRecursion.getFrameTuple2d(), cmpOffsetRecursionEffect.getFrameTuple2d(), controllerCurrentICP.getFrameTuple2d(),
               controllerPerfectCMP.getFrameTuple2d(), stanceCMPProjection.getFrameTuple2d(), footstepRecursionMultiplierCalculator.getCurrentStateProjectionMultiplier());
      }
      else
      {
         solver.compute(finalICPRecursion.getFrameTuple2d(), null, controllerCurrentICP.getFrameTuple2d(), controllerPerfectCMP.getFrameTuple2d(),
               stanceCMPProjection.getFrameTuple2d(), footstepRecursionMultiplierCalculator.getCurrentStateProjectionMultiplier());
      }

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         solver.getFootstepSolutionLocation(i, locationSolution);
         footstepSolutions.get(i).set(locationSolution);
      }
   }

   private final FramePoint2d blankFramePoint = new FramePoint2d(worldFrame);
   private void doFeedbackOnlyControl()
   {
      solver.submitProblemConditions(0, false, true, false);
      solver.setFeedbackConditions(scaledFeedbackWeight.getDoubleValue(), feedbackGain.getDoubleValue());

      solver.compute(controllerDesiredICP.getFrameTuple2d(), null, controllerCurrentICP.getFrameTuple2d(), controllerPerfectCMP.getFrameTuple2d(), blankFramePoint, 1.0);
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
         double entryMutliplier = footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(footstepIndex);
         double exitMutliplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(footstepIndex);

         footstepRecursionMultiplier = entryMutliplier + exitMutliplier;
      }
      else
      {
         footstepRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(footstepIndex);
      }

      solver.setFootstepAdjustmentConditions(footstepIndex, footstepRecursionMultiplier, footstepWeight, upcomingFootsteps.get(footstepIndex));
   }

   private final FramePoint2d finalEndingICP2d = new FramePoint2d(worldFrame);
   private void computeFinalICPRecursion()
   {
      int stepIndexToPoll;
      if (isInTransfer.getBooleanValue())
         stepIndexToPoll = numberOfFootstepsToConsider.getIntegerValue() + 2;
      else
         stepIndexToPoll = numberOfFootstepsToConsider.getIntegerValue() + 1;

      FramePoint finalEndingICP = referenceCMPsCalculator.getEntryCMPs().get(stepIndexToPoll).getFrameTuple();
      finalEndingICP.changeFrame(worldFrame);
      finalEndingICP2d.setByProjectionOntoXYPlane(finalEndingICP);

      finalICP.set(finalEndingICP2d);
      finalICPRecursion.set(finalEndingICP2d);
      finalICPRecursion.scale(footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());
   }

   private final FramePoint2d previousStanceExitCMP2d = new FramePoint2d(worldFrame);
   private final FramePoint2d stanceEntryCMP2d = new FramePoint2d(worldFrame);
   private final FramePoint2d stanceExitCMP2d = new FramePoint2d(worldFrame);
   private void computeStanceCMPProjection()
   {
      footstepRecursionMultiplierCalculator.computeRemainingProjectionMultipliers(timeRemainingInState.getDoubleValue(), useTwoCMPsInControl.getBooleanValue(),
            isInTransfer.getBooleanValue(), isInTransferEntry.getBooleanValue());

      if (useTwoCMPsInControl.getBooleanValue())
      {
         if (isInTransfer.getBooleanValue())
         {
            FramePoint previousStanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(1).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(1).getFrameTuple();

            previousStanceExitCMP2d.setByProjectionOntoXYPlane(previousStanceExitCMP);
            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);
         }
         else
         {
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();

            previousStanceExitCMP2d.setToZero();
            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);
         }
      }
      else
      {
         if (isInTransfer.getBooleanValue())
         {
            FramePoint previousStanceExitCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getEntryCMPs().get(1).getFrameTuple();

            previousStanceExitCMP2d.setByProjectionOntoXYPlane(previousStanceExitCMP);
            stanceEntryCMP2d.setToZero();
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);
         }
         else
         {
            FramePoint stanceExitCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();

            previousStanceExitCMP2d.setToZero();
            stanceEntryCMP2d.setToZero();
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);
         }
      }

      double previousExitMultiplier = footstepRecursionMultiplierCalculator.getRemainingPreviousStanceExitCMPProjectionMultiplier();
      double entryMultiplier = footstepRecursionMultiplierCalculator.getRemainingStanceEntryCMPProjectionMultiplier();
      double exitMultiplier = footstepRecursionMultiplierCalculator.getRemainingStanceExitCMPProjectionMultiplier();

      double currentStateProjectionMultiplier = footstepRecursionMultiplierCalculator.getCurrentStateProjectionMultiplier();

      previousExitMultiplier *= currentStateProjectionMultiplier;
      entryMultiplier *= currentStateProjectionMultiplier;
      exitMultiplier *= currentStateProjectionMultiplier;

      entryMultiplier += footstepRecursionMultiplierCalculator.getStanceEntryCMPProjectionMultiplier();
      exitMultiplier += footstepRecursionMultiplierCalculator.getStanceExitCMPProjectionMultiplier();

      this.stanceEntryCMP.set(stanceEntryCMP2d);
      this.stanceExitCMP.set(stanceExitCMP2d);
      this.previousStanceExitCMP.set(previousStanceExitCMP2d);

      previousStanceExitCMP2d.scale(previousExitMultiplier);
      stanceEntryCMP2d.scale(entryMultiplier);
      stanceExitCMP2d.scale(exitMultiplier);

      stanceCMPProjection.set(previousStanceExitCMP2d);
      stanceCMPProjection.add(stanceEntryCMP2d);
      stanceCMPProjection.add(stanceExitCMP2d);
   }

   private final FramePoint2d totalOffsetEffect = new FramePoint2d();
   private void computeCMPOffsetRecursionEffect()
   {
      computeTwoCMPOffsets();

      totalOffsetEffect.setToZero();
      cmpOffsetRecursionEffect.setToZero();

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         totalOffsetEffect.set(yoExitOffsets.get(i).getFrameTuple2d());
         totalOffsetEffect.scale(footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(i));

         cmpOffsetRecursionEffect.add(totalOffsetEffect);

         totalOffsetEffect.set(yoEntryOffsets.get(i).getFrameTuple2d());
         totalOffsetEffect.scale(footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(i));

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

         FrameVector2d entryOffset = entryOffsets.get(i);
         FrameVector2d exitOffset = exitOffsets.get(i);

         entryOffset.setToZero(worldFrame);
         exitOffset.setToZero(worldFrame);

         entryOffset.setY(stepSide.negateIfLeftSide(icpPlannerParameters.getEntryCMPInsideOffset()));
         entryOffset.setX(icpPlannerParameters.getEntryCMPForwardOffset());
         exitOffset.setY(stepSide.negateIfLeftSide(icpPlannerParameters.getExitCMPInsideOffset()));
         exitOffset.setX(icpPlannerParameters.getExitCMPForwardOffset());

         entryOffset.applyTransform(transform);
         exitOffset.applyTransform(transform);

         yoEntryOffsets.get(i).set(entryOffset);
         yoExitOffsets.get(i).set(exitOffset);
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
         double remainingTime;
         if (isInTransfer.getBooleanValue())
         {
            if (timeInCurrentState.getDoubleValue() < doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue())
            {
               isInTransferEntry.set(true);
            }
            else
            {
               isInTransferEntry.set(false);
            }
            remainingTime = doubleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
         }
         else
         {
            remainingTime = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
         }

         remainingTime = Math.max(icpOptimizationParameters.getMinimumTimeRemaining(), remainingTime);
         timeRemainingInState.set(remainingTime);
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

   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider.getIntegerValue();
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controllerFeedbackCMP.getFrameTuple2d(desiredCMPToPack);
   }

   public void getFootstepSolution(int footstepIndex, FramePoint2d footstepSolutionToPack)
   {
      footstepSolutions.get(footstepIndex).getFrameTuple2d(footstepSolutionToPack);
   }
}
