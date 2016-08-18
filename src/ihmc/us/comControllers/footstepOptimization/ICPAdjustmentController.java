package ihmc.us.comControllers.footstepOptimization;

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

public class ICPAdjustmentController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER = 5;
   private static final double MINIMUM_REMAINING_TIME = 0.001;

   private final String namePrefix = "icpAdjustmentCalculator";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable useTwoCMPs = new BooleanYoVariable("useTwoCMPsPerSupport", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable scaleFirstStepWeightWithTime = new BooleanYoVariable("scaleFirstStepWeightWithTime", registry);
   private final BooleanYoVariable scaleFeedbackWeightWithGains = new BooleanYoVariable("scaleFeedbackWeightWithGains", registry);

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();

   private final IntegerYoVariable numberOfFootstepsInPlan = new IntegerYoVariable("numberOfFootstepsInPlan", registry);
   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);
   private final IntegerYoVariable maxNumberOfFootstepsToConsider = new IntegerYoVariable("maxNumberOfFootstepsToConsider", registry);

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable(namePrefix + "TimeSpentOnExitCMPInPercentOfStepTime", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable("remainingTime", registry);
   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);

   private final ArrayList<DoubleYoVariable> stepWeights = new ArrayList<>();

   private final ArrayList<YoFramePoint2d> footstepSolutionLocations = new ArrayList<>();
   private final YoFrameVector2d cmpFeedbackDifferenceSolution;
   private final YoFramePoint2d cmpFeedbackSolution;
   private final DoubleYoVariable costToGo;

   private final DoubleYoVariable feedbackWeight = new DoubleYoVariable("feedbackWeight", registry);
   private final DoubleYoVariable effectiveFeedbackWeight = new DoubleYoVariable("effectiveFeedbackWeight", registry);
   private final DoubleYoVariable effectiveFirstStepWeight = new DoubleYoVariable("effectiveFirstStepWeight", registry);

   private final DoubleYoVariable omega0;
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable feedbackGain = new DoubleYoVariable("feedbackGain", registry);
   private final DoubleYoVariable effectiveFeedbackGain = new DoubleYoVariable("effectiveFeedbackGain", registry); // kappa

   private final YoFramePoint2d finalICPRecursion = new YoFramePoint2d("finalICPRecursion", worldFrame, registry);

   private final YoFramePoint2d currentICP = new YoFramePoint2d("currentICP", worldFrame, registry);
   private final YoFramePoint2d perfectCMP = new YoFramePoint2d("perfectCMP", worldFrame, registry);

   private final CapturePointPlannerParameters icpPlannerParameters;

   private final FramePoint desiredICP = new FramePoint();
   private final FrameVector desiredICPVelocity = new FrameVector();

   private final TargetTouchdownICPCalculator targetTouchdownICPCalculator;
   private final StepRecursionMultiplierCalculator stepRecursionMultiplierCalculator;
   private final ICPAdjustmentSolver icpAdjustmentSolver;

   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   public ICPAdjustmentController(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters icpPlannerParameters, DoubleYoVariable omega0, YoVariableRegistry parentRegistry)
   {
      this.omega0 = omega0;
      this.icpPlannerParameters = icpPlannerParameters;

      useTwoCMPs.set(icpPlannerParameters.useTwoCMPsPerSupport());
      useFeedback.set(true);
      scaleFirstStepWeightWithTime.set(false);
      scaleFeedbackWeightWithGains.set(false);

      numberOfFootstepsToConsider.set(2);
      maxNumberOfFootstepsToConsider.set(MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER);

      targetTouchdownICPCalculator = new TargetTouchdownICPCalculator(omega0, registry);
      stepRecursionMultiplierCalculator = new StepRecursionMultiplierCalculator(omega0, maxNumberOfFootstepsToConsider.getIntegerValue(), registry);
      icpAdjustmentSolver = new ICPAdjustmentSolver(maxNumberOfFootstepsToConsider.getIntegerValue(), registry);

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            numberOfFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      for (int i = 0; i < MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER; i++)
      {
         stepWeights.add(new DoubleYoVariable("step" + i + "Weight", registry));

         footstepSolutionLocations.add(new YoFramePoint2d("footstep" + i + "SolutionLocation", worldFrame, registry));
      }

      cmpFeedbackDifferenceSolution = new YoFrameVector2d("cmpFeedbackDifferenceSolution", worldFrame, registry);
      cmpFeedbackSolution = new YoFramePoint2d("cmpFeedbackSolution", worldFrame, registry);
      costToGo = new DoubleYoVariable("costToGo", registry);

      parentRegistry.addChild(registry);
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
      numberOfFootstepsInPlan.set(0);
   }

   public void addFootstep(Footstep footstep)
   {
      referenceCMPsCalculator.addUpcomingFootstep(footstep);
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

   public void initializeForDoubleSupport(double initialTime)
   {
      isInDoubleSupport.set(true);
      this.initialTime.set(initialTime);
   }

   public void initializeForSingleSupport(double initialTime)
   {
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


      if (useTwoCMPs.getBooleanValue())
      {
         double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
         double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

         computeRemainingTimeInState(totalTimeSpentOnExitCMP);

         perfectCMP.set(referenceCMPsCalculator.getExitCMPs().get(0).getFramePoint2dCopy());
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(totalTimeSpentOnExitCMP, totalTimeSpentOnEntryCMP,
               numberOfFootstepsToConsider.getIntegerValue(), useTwoCMPs.getBooleanValue());
      }
      else
      {
         computeRemainingTimeInState(steppingDuration);

         perfectCMP.set(referenceCMPsCalculator.getEntryCMPs().get(0).getFramePoint2dCopy());
         stepRecursionMultiplierCalculator.computeRecursionMultipliers(steppingDuration, numberOfFootstepsToConsider.getIntegerValue(), useTwoCMPs.getBooleanValue());
      }

      targetTouchdownICPCalculator.computeTargetTouchdownICP(remainingTime.getDoubleValue(), currentICP, perfectCMP.getFramePoint2dCopy());

      computeEffectiveFeedbackWeight(scaleFeedbackWeightWithGains.getBooleanValue());
      computeEffectiveFirstStepWeight(remainingTime.getDoubleValue(), steppingDuration, scaleFirstStepWeightWithTime.getBooleanValue());
      computeTargetICPRecursionBackward();
      computeEffectiveFeedbackGain(remainingTime.getDoubleValue());

      submitInformationToSolver();

      icpAdjustmentSolver.solve();

      getSolutions();
   }

   private void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   private void computeRemainingTimeInState(double totalTime)
   {
      double remainingTime = totalTime - timeInCurrentState.getDoubleValue();
      remainingTime = Math.max(remainingTime, MINIMUM_REMAINING_TIME);

      this.remainingTime.set(remainingTime);
   }

   private void clipNumberOfStepsToConsider()
   {
      numberOfFootstepsToConsider.set(Math.min(numberOfFootstepsInPlan.getIntegerValue(), numberOfFootstepsToConsider.getIntegerValue()));
      numberOfFootstepsToConsider.set(Math.min(maxNumberOfFootstepsToConsider.getIntegerValue(), numberOfFootstepsToConsider.getIntegerValue()));
   }

   private void computeEffectiveFeedbackWeight(boolean scaleFeedbackWeightWithGains)
   {
      if (scaleFeedbackWeightWithGains)
      {
         double desiredFeedbackWeight = feedbackWeight.getDoubleValue();
         double feedbackGainEffect = 1 + feedbackGain.getDoubleValue() / omega0.getDoubleValue();

         effectiveFeedbackWeight.set(desiredFeedbackWeight / Math.pow(feedbackGainEffect, 2));
      }
      else
      {
         effectiveFeedbackWeight.set(feedbackWeight.getDoubleValue());
      }
   }

   private void computeEffectiveFirstStepWeight(double timeRemaining, double steppingDuration, boolean scaleFirstStepWeightWithTime)
   {
      double alpha = 1.0;
      double firstStepWeight = stepWeights.get(0).getDoubleValue();

      if (scaleFirstStepWeightWithTime)
         alpha = steppingDuration / timeRemaining;

      effectiveFirstStepWeight.set(alpha * firstStepWeight);
   }

   private void computeEffectiveFeedbackGain(double timeRemaining)
   {
      double effectiveFeedbackGain = -Math.exp(omega0.getDoubleValue() * timeRemaining) / (1.0 + feedbackGain.getDoubleValue() / omega0.getDoubleValue()); // kappa
      this.effectiveFeedbackGain.set(effectiveFeedbackGain);
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

   private final FramePoint2d targetTouchdownICP = new FramePoint2d();
   private final FramePoint footstepLocation = new FramePoint();
   private final FramePoint2d footstepLocation2d = new FramePoint2d();
   private final Quat4d footstepOrientation = new Quat4d();

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final FrameVector2d entryOffset = new FrameVector2d();
   private final FrameVector2d exitOffset = new FrameVector2d();

   private void submitInformationToSolver()
   {
      icpAdjustmentSolver.setProblemConditions(numberOfFootstepsToConsider.getIntegerValue(), useFeedback.getBooleanValue(), useTwoCMPs.getBooleanValue());
      icpAdjustmentSolver.reset();

      boolean useTwoCMPs = this.useTwoCMPs.getBooleanValue();

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         Footstep upcomingFootstep = upcomingFootsteps.get(i);
         RobotSide stepSide = upcomingFootstep.getRobotSide();
         upcomingFootstep.getPositionIncludingFrame(footstepLocation);
         upcomingFootstep.getOrientationInWorldFrame(footstepOrientation);

         transform.zeroTranslation();
         transform.setRotation(footstepOrientation);

         footstepLocation.changeFrame(worldFrame);
         footstepLocation.setXY(footstepLocation2d);

         entryOffset.setToZero(worldFrame);
         exitOffset.setToZero(worldFrame);

         entryOffset.setY(stepSide.negateIfLeftSide(icpPlannerParameters.getEntryCMPInsideOffset()));
         entryOffset.setX(icpPlannerParameters.getEntryCMPForwardOffset());
         exitOffset.setY(stepSide.negateIfLeftSide(icpPlannerParameters.getExitCMPInsideOffset()));
         exitOffset.setX(icpPlannerParameters.getExitCMPForwardOffset());

         entryOffset.applyTransform(transform);
         exitOffset.applyTransform(transform);

         icpAdjustmentSolver.setReferenceFootstepLocation(i, footstepLocation2d, entryOffset, exitOffset);

         if (useTwoCMPs)
         {
            double entryMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionEntryMultiplier(i, useTwoCMPs);
            double exitMultiplier = stepRecursionMultiplierCalculator.getTwoCMPRecursionExitMultiplier(i, useTwoCMPs);
            icpAdjustmentSolver.setFootstepRecursionMultipliers(i, entryMultiplier, exitMultiplier);
         }
         else
         {
            icpAdjustmentSolver.setFootstepRecursionMultipliers(i, stepRecursionMultiplierCalculator.getOneCMPRecursionMultiplier(i, useTwoCMPs));
         }

         if (i == 0)
            icpAdjustmentSolver.setFootstepWeight(i, effectiveFirstStepWeight.getDoubleValue());
         else
            icpAdjustmentSolver.setFootstepWeight(i, stepWeights.get(i).getDoubleValue());
      }

      icpAdjustmentSolver.setFeedbackWeight(feedbackWeight.getDoubleValue());
      icpAdjustmentSolver.setEffectiveFeedbackGain(effectiveFeedbackGain.getDoubleValue());
      icpAdjustmentSolver.setFinalICPRecursion(finalICPRecursion.getFramePoint2dCopy());


      targetTouchdownICPCalculator.getTargetTouchdownICP(targetTouchdownICP);
      icpAdjustmentSolver.setTargetTouchdownICP(targetTouchdownICP);
      icpAdjustmentSolver.setPerfectCMP(perfectCMP.getFramePoint2dCopy());

      icpAdjustmentSolver.computeMatrices();
   }

   private final FramePoint2d footstepSolutionLocation = new FramePoint2d();
   private final FramePoint2d cmpFeedbackSolutionTmp = new FramePoint2d();
   private final FrameVector2d cmpFeedbackDifferenceSolutionTmp = new FrameVector2d();

   private void getSolutions()
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
}
