package ihmc.us.comControllers.controllers.footstepOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
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

   private final boolean useTwoCMPsPerSupport;

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();

   private final IntegerYoVariable numberOfFootstepsInPlan = new IntegerYoVariable("numberOfFootstepsInPlan", registry);
   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);
   private final IntegerYoVariable maxNumberOfFootstepsToConsider = new IntegerYoVariable("maxNumberOfFootstepsToConsider", registry);

   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable(namePrefix + "TimeSpentOnExitCMPInPercentOfStepTime", registry);
   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);


   private final ArrayList<DoubleYoVariable> toeRecursionMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> heelRecursionMultipliers = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> stepWeights = new ArrayList<>();

   private final ArrayList<YoFramePoint2d> footstepSolutionLocations = new ArrayList<>();
   private final YoFrameVector2d cmpFeedbackDifferenceSolution;
   private final YoFramePoint2d cmpFeedbackSolution;
   private final DoubleYoVariable costToGo;

   private final DoubleYoVariable feedbackWeight;
   private final DoubleYoVariable effectiveFeedbackWeight;
   private final DoubleYoVariable effectiveFirstStepWeight = new DoubleYoVariable("effectiveFirstStepWeight", registry);

   private final DoubleYoVariable omega0;
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable feedbackGain = new DoubleYoVariable("feedbackGain", registry);
   private final DoubleYoVariable effectiveFeedbackGain = new DoubleYoVariable("effectiveFeedbackGain", registry); // kappa
   private final DoubleYoVariable remainingDurationProjection = new DoubleYoVariable("remainingDurationProjection", registry); // phi

   private final YoFramePoint2d cmpProjectionForward = new YoFramePoint2d("cmpProjectionForward", worldFrame, registry);
   private final YoFramePoint2d targetICPRecursion = new YoFramePoint2d("targetICPRecursion", worldFrame, registry);

   private final CapturePointPlannerParameters icpPlannerParameters;

   private final FramePoint desiredICP = new FramePoint();
   private final FrameVector desiredICPVelocity = new FrameVector();

   private final FramePoint currentICP = new FramePoint();

   private final ICPAdjustmentSolver icpAdjustmentSolver;
   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   public ICPAdjustmentController(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters icpPlannerParameters, DoubleYoVariable omega0, YoVariableRegistry parentRegistry)
   {
      this.omega0 = omega0;
      this.icpPlannerParameters = icpPlannerParameters;
      useTwoCMPsPerSupport = icpPlannerParameters.useTwoCMPsPerSupport();

      numberOfFootstepsToConsider.set(2);
      maxNumberOfFootstepsToConsider.set(MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER);

      icpAdjustmentSolver = new ICPAdjustmentSolver(maxNumberOfFootstepsToConsider.getIntegerValue(), registry);

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            numberOfFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      for (int i = 0; i < MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER; i++)
      {
         toeRecursionMultipliers.add(new DoubleYoVariable("step" + i + "ToeRecursionMultiplier", registry));
         heelRecursionMultipliers.add(new DoubleYoVariable("step" + i + "HeelRecursionMultiplier", registry));
         stepWeights.add(new DoubleYoVariable("step" + i + "Weight", registry));

         footstepSolutionLocations.add(new YoFramePoint2d("footstep" + i + "SolutionLocation", worldFrame, registry));
      }

      feedbackWeight = new DoubleYoVariable("feedbackWeight", registry);
      effectiveFeedbackWeight = new DoubleYoVariable("effectiveFeedbackWeight", registry);

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

   public void setCurrentICP(FramePoint currentICP)
   {
      currentICP.changeFrame(worldFrame);

      this.currentICP.set(currentICP);
   }

   public void clearPlan()
   {
      referenceCMPsCalculator.clear();
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
      computeRecursionMultipliers();
   }


   private final FramePoint2d footstepSolutionLocation = new FramePoint2d();
   private final FrameVector2d cmpFeedbackDifferenceSolutionTmp = new FrameVector2d();

   public void compute(double time, FramePoint2d currentICP)
   {
      computeTimeInCurrentState(time);
      computeEffectiveFeedbackWeight();

      int numberOfFootstepsToConsider = this.numberOfFootstepsToConsider.getIntegerValue();

      if (numberOfFootstepsToConsider > numberOfFootstepsInPlan.getIntegerValue())
         numberOfFootstepsToConsider = numberOfFootstepsInPlan.getIntegerValue();
      if (numberOfFootstepsToConsider > maxNumberOfFootstepsToConsider.getIntegerValue())
      {
         numberOfFootstepsToConsider = maxNumberOfFootstepsToConsider.getIntegerValue();
         this.numberOfFootstepsToConsider.set(maxNumberOfFootstepsToConsider.getIntegerValue());
      }

      double steppingDuration = doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue();
      double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();

      double remainingTime = totalTimeSpentOnExitCMP - timeInCurrentState.getDoubleValue();

      if (remainingTime < MINIMUM_REMAINING_TIME)
         remainingTime = MINIMUM_REMAINING_TIME;

      computeEffectiveFirstStepWeight(remainingTime, steppingDuration);
      computeCMPProjectionForward(remainingTime);
      computeTargetICPRecursionBackward();

      remainingDurationProjection.set(1 + Math.exp(omega0.getDoubleValue() * remainingTime)); // phi

      double effectiveFeedbackGain = -remainingDurationProjection.getDoubleValue() / (1 + feedbackGain.getDoubleValue() / omega0.getDoubleValue()); // kappa
      this.effectiveFeedbackGain.set(effectiveFeedbackGain);

      submitInformation(currentICP);

      icpAdjustmentSolver.solve();

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         icpAdjustmentSolver.getFootstepSolutionLocation(i, footstepSolutionLocation);
         footstepSolutionLocations.get(i).set(footstepSolutionLocation);
      }

      icpAdjustmentSolver.getCMPFeedbackDifference(cmpFeedbackDifferenceSolutionTmp);
      cmpFeedbackDifferenceSolution.set(cmpFeedbackDifferenceSolutionTmp);

      FramePoint currentToeOffCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();
      currentToeOffCMP.changeFrame(worldFrame);
      currentToeOffCMP.getXYPlaneDistance(currentToeOffCMP2d);

      cmpFeedbackSolution.set(currentToeOffCMP2d);
      cmpFeedbackSolution.add(cmpFeedbackDifferenceSolutionTmp);

      costToGo.set(icpAdjustmentSolver.getCostToGo());
   }

   private void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   private void computeRecursionMultipliers()
   {
      double doubleSupportDuration = this.doubleSupportDuration.getDoubleValue();
      double singleSupportDuration = this.singleSupportDuration.getDoubleValue();

      double steppingDuration = doubleSupportDuration + singleSupportDuration;
      double totalTimeSpentOnExitCMP = steppingDuration * exitCMPDurationInPercentOfStepTime.getDoubleValue();
      double totalTimeSpentOnEntryCMP = steppingDuration * (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue());

      double firstToeRecursionMultiplier = Math.exp(-omega0.getDoubleValue() * totalTimeSpentOnEntryCMP) * (1 - Math.exp(-omega0.getDoubleValue() * totalTimeSpentOnExitCMP));
      double secondToeRecursionMultiplier = Math.exp(-omega0.getDoubleValue() * (steppingDuration + totalTimeSpentOnEntryCMP)) *
            (1 - Math.exp(-omega0.getDoubleValue() * totalTimeSpentOnExitCMP));

      double firstHeelRecursionMultiplier = (1 - Math.exp(-omega0.getDoubleValue() * totalTimeSpentOnEntryCMP));
      double secondHeelRecursionMultiplier = Math.exp(-omega0.getDoubleValue() * steppingDuration) *
            (1 - Math.exp(-omega0.getDoubleValue() * totalTimeSpentOnEntryCMP));

      toeRecursionMultipliers.get(0).set(firstToeRecursionMultiplier);
      toeRecursionMultipliers.get(1).set(secondToeRecursionMultiplier);

      heelRecursionMultipliers.get(0).set(firstHeelRecursionMultiplier);
      heelRecursionMultipliers.get(1).set(secondHeelRecursionMultiplier);
   }

   private void computeEffectiveFeedbackWeight()
   {
      double desiredFeedbackWeight = feedbackWeight.getDoubleValue();
      double feedbackGainEffect = 1 + feedbackGain.getDoubleValue() / omega0.getDoubleValue();

      effectiveFeedbackGain.set(desiredFeedbackWeight / Math.pow(feedbackGainEffect, 2));
   }

   private void computeEffectiveFirstStepWeight(double timeRemaining, double steppingDuration)
   {
      double firstStepWeight = stepWeights.get(0).getDoubleValue();
      double alpha = steppingDuration / timeRemaining;

      effectiveFirstStepWeight.set(alpha * firstStepWeight);
   }

   private final FramePoint2d currentToeOffCMP2d = new FramePoint2d();
   private void computeCMPProjectionForward(double timeRemaining)
   {
      double cmpProjectionForward = Math.exp(omega0.getDoubleValue() * timeRemaining);
      FramePoint currentToeOffCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();
      currentToeOffCMP.changeFrame(worldFrame);
      currentToeOffCMP.getXYPlaneDistance(currentToeOffCMP2d);

      this.cmpProjectionForward.set(currentToeOffCMP2d);
      this.cmpProjectionForward.scale(cmpProjectionForward);
   }

   private final FramePoint2d targetEndingICP2d = new FramePoint2d();
   private void computeTargetICPRecursionBackward()
   {
      double totalTimeForRecursion = numberOfFootstepsToConsider.getIntegerValue() * (doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());
      double projectionBackward = Math.exp(-omega0.getDoubleValue() * totalTimeForRecursion);

      FramePoint targetEndingICP = referenceCMPsCalculator.getEntryCMPs().get(numberOfFootstepsToConsider.getIntegerValue()).getFrameTuple();
      targetEndingICP.changeFrame(worldFrame);
      targetEndingICP.getXYPlaneDistance(targetEndingICP2d);

      targetICPRecursion.set(targetEndingICP2d);
      targetICPRecursion.scale(projectionBackward);
   }

   private final FramePoint footstepLocation = new FramePoint();
   private final FramePoint2d footstepLocation2d = new FramePoint2d();
   private final Quat4d footstepOrientation = new Quat4d();

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final FrameVector2d entryOffset = new FrameVector2d();
   private final FrameVector2d exitOffset = new FrameVector2d();

   private void submitInformation(FramePoint2d currentICP)
   {
      icpAdjustmentSolver.setProblemConditions(numberOfFootstepsToConsider.getIntegerValue());
      icpAdjustmentSolver.reset();

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

         icpAdjustmentSolver.setFootstepRecursionMultipliers(i, heelRecursionMultipliers.get(i).getDoubleValue(), toeRecursionMultipliers.get(i).getDoubleValue());

         if (i == 0)
            icpAdjustmentSolver.setFootstepWeight(i, effectiveFirstStepWeight.getDoubleValue());
         else
            icpAdjustmentSolver.setFootstepWeight(i, stepWeights.get(i).getDoubleValue());
      }

      icpAdjustmentSolver.setTargetICPRecursion(targetICPRecursion.getFramePoint2dCopy());
      icpAdjustmentSolver.setCMPProjectionForward(cmpProjectionForward.getFramePoint2dCopy());
      icpAdjustmentSolver.setFeedbackWeight(feedbackWeight.getDoubleValue());
      icpAdjustmentSolver.setEffectiveFeedbackGain(effectiveFeedbackGain.getDoubleValue());
      icpAdjustmentSolver.setRemainingDurationProjection(remainingDurationProjection.getDoubleValue());
      icpAdjustmentSolver.setCurrentICPValue(currentICP);

      icpAdjustmentSolver.assembleCostMatrices();
   }
}
