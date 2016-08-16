package ihmc.us.comControllers.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ReferenceCentroidalMomentumPivotLocationsCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class ICPAdjustmentController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER = 5;

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
   private final DoubleYoVariable effectiveFirstStepWeight = new DoubleYoVariable("effectiveFirstStepWeight", registry);

   private final DoubleYoVariable omega0;
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable feedbackGain = new DoubleYoVariable("feedbackGain", registry);
   private final DoubleYoVariable effectiveFeedbackGain = new DoubleYoVariable("effectiveFeedbackGain", registry);
   private final DoubleYoVariable remainingDurationProjection = new DoubleYoVariable("remainingDurationProjection", registry);

   private final FramePoint desiredICP = new FramePoint();
   private final FrameVector desiredICPVelocity = new FrameVector();

   private final FramePoint currentICP = new FramePoint();

   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   public ICPAdjustmentController(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters icpPlannerParameters, DoubleYoVariable omega0, YoVariableRegistry parentRegistry)
   {
      this.omega0 = omega0;
      useTwoCMPsPerSupport = icpPlannerParameters.useTwoCMPsPerSupport();

      numberOfFootstepsToConsider.set(2);
      maxNumberOfFootstepsToConsider.set(MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER);

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            numberOfFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      for (int i = 0; i < MAX_NUMBER_OF_FOOTSTEPS_TO_CONSIDER; i++)
      {
         toeRecursionMultipliers.add(new DoubleYoVariable("step" + i + "ToeRecursionMultiplier", registry));
         heelRecursionMultipliers.add(new DoubleYoVariable("step" + i + "HeelRecursionMultiplier", registry));
         stepWeights.add(new DoubleYoVariable("step" + i + "Weight", registry));
      }

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

   public void compute(double time)
   {
      computeTimeInCurrentState(time);

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
      remainingDurationProjection.set(1 + Math.exp(omega0.getDoubleValue() * remainingTime));

      double effectiveFeedbackGain = -remainingDurationProjection.getDoubleValue() / (1 + feedbackGain.getDoubleValue() / omega0.getDoubleValue());
      this.effectiveFeedbackGain.set(effectiveFeedbackGain);
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
}
