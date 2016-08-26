package us.ihmc.comControllers.footstepOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ReferenceCentroidalMomentumPivotLocationsCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.List;

public class ICPAdjustmentController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpAdjustmentCalculator";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable useTwoCMPs = new BooleanYoVariable("useTwoCMPsPerSupport", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable useStepAdjustment = new BooleanYoVariable("useStepAdjustment", registry);
   private final BooleanYoVariable scaleFirstStepWeightWithTime = new BooleanYoVariable("scaleFirstStepWeightWithTime", registry);

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<FrameVector2d> entryOffsets = new ArrayList<>();
   private final ArrayList<FrameVector2d> exitOffsets = new ArrayList<>();

   private final IntegerYoVariable numberOfFootstepsInPlan = new IntegerYoVariable("numberOfFootstepsInPlan", registry);
   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);
   private final IntegerYoVariable yoMaxNumberOfFootstepsToConsider = new IntegerYoVariable("maxNumberOfFootstepsToConsider", registry);

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable(namePrefix + "TimeSpentOnExitCMPInPercentOfStepTime", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable("remainingTime", registry);
   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);
   private final BooleanYoVariable isStanding = new BooleanYoVariable("isStanding", registry);

   private final ArrayList<DoubleYoVariable> stepWeights = new ArrayList<>();

   private final ArrayList<YoFramePoint2d> footstepSolutionLocations = new ArrayList<>();
   private final YoFrameVector2d cmpFeedbackDifferenceSolution;
   private final YoFramePoint2d cmpFeedbackSolution;
   private final DoubleYoVariable costToGo;

   private final DoubleYoVariable feedbackWeight = new DoubleYoVariable("feedbackWeight", registry);
   private final DoubleYoVariable feedbackDynamicEffect = new DoubleYoVariable("feedbackDynamicEffect", registry);
   private final DoubleYoVariable effectiveFirstStepWeight = new DoubleYoVariable("effectiveFirstStepWeight", registry);

   private final DoubleYoVariable omega0;
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);

   private final YoFramePoint2d finalICPRecursion = new YoFramePoint2d("finalICPRecursion", worldFrame, registry);
   private final YoFramePoint2d twoCMPOffsetEffect = new YoFramePoint2d("twoCMPOffsetEffect", worldFrame, registry);
   private final YoFramePoint2d desiredICPToHold = new YoFramePoint2d("desiredICPToHold", worldFrame, registry);

   private final YoFramePoint2d currentICP = new YoFramePoint2d("currentICP", worldFrame, registry);
   private final YoFramePoint2d perfectCMP = new YoFramePoint2d("perfectCMP", worldFrame, registry);

   private final CapturePointPlannerParameters icpPlannerParameters;

   private final FramePoint desiredICP = new FramePoint();
   private final FrameVector desiredICPVelocity = new FrameVector();

   private final TargetTouchdownICPCalculator targetTouchdownICPCalculator;
   private final StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator;
   private final ICPAdjustmentSolver icpAdjustmentSolver;

   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final double minimumRemainingTime;

   public ICPAdjustmentController(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters icpPlannerParameters, ICPAdjustmentControllerParameters icpControllerParameters, DoubleYoVariable omega0,
         YoVariableRegistry parentRegistry)
   {
      this.omega0 = omega0;
      this.icpPlannerParameters = icpPlannerParameters;

      useTwoCMPs.set(icpPlannerParameters.useTwoCMPsPerSupport());
      useFeedback.set(icpControllerParameters.useFeedback());
      scaleFirstStepWeightWithTime.set(icpControllerParameters.scaleFirstStepWithTime());

      minimumRemainingTime = icpControllerParameters.minimumRemainingTime();

      numberOfFootstepsToConsider.set(icpControllerParameters.getNumberOfStepsToConsider());
      yoMaxNumberOfFootstepsToConsider.set(icpControllerParameters.getMaximumNumberOfStepsToConsider());

      targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega0, registry);
      stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega0, yoMaxNumberOfFootstepsToConsider.getIntegerValue(), registry);
      icpAdjustmentSolver = new ICPAdjustmentSolver(icpControllerParameters);

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            yoMaxNumberOfFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      for (int i = 0; i < yoMaxNumberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         DoubleYoVariable stepWeight = new DoubleYoVariable("step" + i + "Weight", registry);
         stepWeight.set(icpControllerParameters.getFootstepWeight());
         stepWeights.add(stepWeight);

         footstepSolutionLocations.add(new YoFramePoint2d("footstep" + i + "SolutionLocation", worldFrame, registry));

         entryOffsets.add(new FrameVector2d(worldFrame));
         exitOffsets.add(new FrameVector2d(worldFrame));
      }
      feedbackWeight.set(icpControllerParameters.getFeedbackWeight());

      cmpFeedbackDifferenceSolution = new YoFrameVector2d("cmpFeedbackDifferenceSolution", worldFrame, registry);
      cmpFeedbackSolution = new YoFramePoint2d("cmpFeedbackSolution", worldFrame, registry);
      costToGo = new DoubleYoVariable("costToGo", registry);

      parentRegistry.addChild(registry);
   }

   public void setFeedbackWeight(double feedbackWeight)
   {
      this.feedbackWeight.set(feedbackWeight);
   }

   public void setFootstepWeight(int foostepIndex, double footstepWeight)
   {
      if (foostepIndex < yoMaxNumberOfFootstepsToConsider.getIntegerValue())
         stepWeights.get(foostepIndex).set(footstepWeight);
   }

   public void setNumberOfFootstepsToConsider(int numberOfFootstepsToConsider)
   {
      this.numberOfFootstepsToConsider.set(numberOfFootstepsToConsider);
      clipNumberOfStepsToConsider();
   }

   public void setDesiredICPValues(FramePoint desiredICP, FrameVector desiredICPVelocity)
   {
      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);

      this.desiredICP.set(desiredICP);
      this.desiredICPVelocity.set(desiredICPVelocity);
   }

   public void clearPlan()
   {
      referenceCMPsCalculator.clear();
      upcomingFootsteps.clear();
      numberOfFootstepsInPlan.set(0);
   }

   public void addFootstep(Footstep footstep)
   {
      referenceCMPsCalculator.addUpcomingFootstep(footstep);
      upcomingFootsteps.add(footstep);
      numberOfFootstepsInPlan.increment();
   }

   public void setDoubleSupportDuration(double time)
   {
      doubleSupportDuration.set(time);
   }

   public void setSingleSupportDuration(double time)
   {
      singleSupportDuration.set(time);
   }

   public void setDesiredICPToHold(FramePoint2d desiredICPToHold)
   {
      desiredICPToHold.changeFrame(worldFrame);
      this.desiredICPToHold.set(desiredICPToHold);
   }

   public void initializeForStanding()
   {
      isStanding.set(true);
      isInDoubleSupport.set(false);
   }

   public void initializeForDoubleSupport(double initialTime)
   {
      isStanding.set(false);
      isInDoubleSupport.set(true);
      this.initialTime.set(initialTime);
   }

   public void initializeForSingleSupport(double initialTime)
   {
      isStanding.set(false);
      isInDoubleSupport.set(false);
      this.initialTime.set(initialTime);
   }

   public void compute(double time, FramePoint2d currentICP)
   {
      currentICP.changeFrame(worldFrame);
      this.currentICP.set(currentICP);

      computeTimeInCurrentState(time);

      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();

      clipNumberOfStepsToConsider();

      if (!isStanding.getBooleanValue())
      {
         if (useTwoCMPs.getBooleanValue())
         {
            double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
            double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

            computeRemainingTimeInState(totalTimeSpentOnExitCMP);

            perfectCMP.set(referenceCMPsCalculator.getExitCMPs().get(0).getFramePoint2dCopy());
            stepRecursionMultiplierCalculator
                  .computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP, numberOfFootstepsToConsider.getIntegerValue(), useTwoCMPs.getBooleanValue());
         }
         else
         {
            computeRemainingTimeInState(steppingDuration);

            perfectCMP.set(referenceCMPsCalculator.getEntryCMPs().get(0).getFramePoint2dCopy());
            stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, numberOfFootstepsToConsider.getIntegerValue(), useTwoCMPs.getBooleanValue());
         }

         targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime.getDoubleValue(), currentICP, perfectCMP.getFramePoint2dCopy());

         computeFeedbackDynamicEffect(remainingTime.getDoubleValue());
         computeEffectiveFirstStepWeight(remainingTime.getDoubleValue(), steppingDuration, scaleFirstStepWeightWithTime.getBooleanValue());
         computeTargetICPRecursionBackward();

         if (useTwoCMPs.getBooleanValue())
         {
            computeTwoCMPOffsets();
            computeEntryAndExitOffsetEffects();
         }
      }

      submitInformationToSolver();

      icpAdjustmentSolver.solve();

      populateSolutionsFromSolver();
   }

   private void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   private void computeRemainingTimeInState(double totalTime)
   {
      double remainingTime = totalTime - timeInCurrentState.getDoubleValue();
      remainingTime = Math.max(remainingTime, minimumRemainingTime);

      this.remainingTime.set(remainingTime);
   }

   private void clipNumberOfStepsToConsider()
   {
      numberOfFootstepsToConsider.set(Math.min(numberOfFootstepsInPlan.getIntegerValue(), numberOfFootstepsToConsider.getIntegerValue()));
      numberOfFootstepsToConsider.set(Math.min(yoMaxNumberOfFootstepsToConsider.getIntegerValue(), numberOfFootstepsToConsider.getIntegerValue()));
   }

   private void computeFeedbackDynamicEffect(double timeRemaining)
   {
      feedbackDynamicEffect.set(1.0 - Math.exp(omega0.getDoubleValue() * timeRemaining));
   }

   private void computeEffectiveFirstStepWeight(double timeRemaining, double steppingDuration, boolean scaleFirstStepWeightWithTime)
   {
      double alpha = 1.0;
      double firstStepWeight = stepWeights.get(0).getDoubleValue();

      if (scaleFirstStepWeightWithTime)
         alpha = steppingDuration / timeRemaining;

      effectiveFirstStepWeight.set(alpha * firstStepWeight);
   }

   private final FramePoint2d finalEndingICP2d = new FramePoint2d();
   private void computeTargetICPRecursionBackward()
   {
      FramePoint finalEndingICP = referenceCMPsCalculator.getEntryCMPs().get(numberOfFootstepsToConsider.getIntegerValue()).getFrameTuple();
      finalEndingICP.changeFrame(worldFrame);
      finalEndingICP.getXYPlaneDistance(finalEndingICP2d);

      finalICPRecursion.set(finalEndingICP2d);
      finalICPRecursion.scale(stepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());
   }

   private final FramePoint2d totalOffsetEffect = new FramePoint2d();
   private void computeEntryAndExitOffsetEffects()
   {
      twoCMPOffsetEffect.setToZero();

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         totalOffsetEffect.set(exitOffsets.get(i));
         totalOffsetEffect.scale(stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs.getBooleanValue()));

         twoCMPOffsetEffect.add(totalOffsetEffect);

         totalOffsetEffect.set(entryOffsets.get(i));
         totalOffsetEffect.scale(stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs.getBooleanValue()));

         twoCMPOffsetEffect.add(totalOffsetEffect);
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
      }
   }


   private final FramePoint2d footstepLocation2d = new FramePoint2d();
   private final FramePoint2d targetTouchdownICP = new FramePoint2d();
   private final FramePoint2d effectiveFinalICPRecursion = new FramePoint2d();

   private void submitInformationToSolver()
   {
      if (isStanding.getBooleanValue())
      {
         icpAdjustmentSolver.setProblemConditions(0, true, false, false);
      }
      else
      {
         icpAdjustmentSolver.setProblemConditions(numberOfFootstepsToConsider.getIntegerValue(), useFeedback.getBooleanValue(), useStepAdjustment.getBooleanValue(),
               useTwoCMPs.getBooleanValue());
         icpAdjustmentSolver.reset();

         boolean useTwoCMPs = this.useTwoCMPs.getBooleanValue();

         for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
         {
            Footstep upcomingFootstep = upcomingFootsteps.get(i);
            upcomingFootstep.getPositionIncludingFrame(footstepLocation);
            upcomingFootstep.getOrientationInWorldFrame(footstepOrientation);

            footstepLocation2d.setToZero(worldFrame);
            footstepLocation2d.setByProjectionOntoXYPlane(footstepLocation);

            icpAdjustmentSolver.setReferenceFootstepLocation(i, footstepLocation2d);

            if (useTwoCMPs)
            {
               double entryMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs);
               double exitMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);
               icpAdjustmentSolver.setFootstepRecursionMultipliers(i, entryMultiplier + exitMultiplier);

               effectiveFinalICPRecursion.set(finalICPRecursion.getFramePoint2dCopy());
               effectiveFinalICPRecursion.add(twoCMPOffsetEffect.getFramePoint2dCopy());
            }
            else
            {
               icpAdjustmentSolver.setFootstepRecursionMultipliers(i, stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs));

               effectiveFinalICPRecursion.set(finalICPRecursion.getFramePoint2dCopy());
            }

            if (i == 0)
               icpAdjustmentSolver.setFootstepWeight(i, effectiveFirstStepWeight.getDoubleValue());
            else
               icpAdjustmentSolver.setFootstepWeight(i, stepWeights.get(i).getDoubleValue());
         }

         icpAdjustmentSolver.setFinalICPRecursion(effectiveFinalICPRecursion);
         icpAdjustmentSolver.setTargetTouchdownICP(targetTouchdownICP);

         targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);
      }

      icpAdjustmentSolver.setFeedbackWeight(feedbackWeight.getDoubleValue());
      icpAdjustmentSolver.setFeedbackDynamicEffect(feedbackDynamicEffect.getDoubleValue());

      icpAdjustmentSolver.setTargetTouchdownICP(desiredICPToHold.getFramePoint2dCopy());
      icpAdjustmentSolver.setPerfectCMP(perfectCMP.getFramePoint2dCopy());

      icpAdjustmentSolver.computeMatrices();
   }

   private final FramePoint2d footstepSolutionLocation = new FramePoint2d();
   private final FramePoint2d cmpFeedbackSolutionTmp = new FramePoint2d();
   private final FrameVector2d cmpFeedbackDifferenceSolutionTmp = new FrameVector2d();

   private void populateSolutionsFromSolver()
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         icpAdjustmentSolver.getFootstepSolutionLocation(i, footstepSolutionLocation);
         footstepSolutionLocations.get(i).set(footstepSolutionLocation);
      }

      icpAdjustmentSolver.getCMPFeedback(cmpFeedbackSolutionTmp);
      icpAdjustmentSolver.getCMPFeedbackDifference(cmpFeedbackDifferenceSolutionTmp);

      cmpFeedbackSolution.set(cmpFeedbackSolutionTmp);
      cmpFeedbackDifferenceSolution.set(cmpFeedbackDifferenceSolutionTmp);

      costToGo.set(icpAdjustmentSolver.getCostToGo());
   }

   public void getCMPFeedback(FramePoint2d cmpFeedback)
   {
      cmpFeedback.changeFrame(worldFrame);
      cmpFeedbackSolution.getFrameTuple2d(cmpFeedback);
   }

   public void getFootstepLocation(int footstepIndex, FramePoint2d footstepLocation)
   {
      footstepSolutionLocations.get(footstepIndex).getFrameTuple2d(footstepLocation);
   }

   private final FramePose footstepPose = new FramePose(worldFrame);
   public void getFootstepLocation(int footstepIndex, Footstep footstep)
   {
      footstep.getPose(footstepPose);
      footstepPose.setXYFromPosition2d(footstepSolutionLocations.get(footstepIndex).getFrameTuple2d());
      footstep.setPose(footstepPose);
   }

   public void getFootstepLocations(List<FramePoint2d> footstepLocations)
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
         getFootstepLocation(i, footstepLocations.get(i));
   }

   public void getAdjustedFootsteps(List<Footstep> footsteps)
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
         getFootstepLocation(i, footsteps.get(i));
   }
}
