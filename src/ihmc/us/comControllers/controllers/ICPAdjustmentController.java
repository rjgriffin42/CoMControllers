package ihmc.us.comControllers.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ReferenceCentroidalMomentumPivotLocationsCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class ICPAdjustmentController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpAdjustmentCalculator";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final boolean useTwoCMPsPerSupport;

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();

   private final IntegerYoVariable numberOfFootstepsInPlan = new IntegerYoVariable("numberOfFootstepsInPlan", registry);
   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);

   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);

   private final FramePoint desiredICP = new FramePoint();
   private final FrameVector desiredICPVelocity = new FrameVector();

   private final FramePoint currentICP = new FramePoint();

   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   public ICPAdjustmentController(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         CapturePointPlannerParameters icpPlannerParameters, YoVariableRegistry parentRegistry)
   {
      useTwoCMPsPerSupport = icpPlannerParameters.useTwoCMPsPerSupport();

      numberOfFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());

      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            numberOfFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

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

   public void initializeForDoubleSupport()
   {
      isInDoubleSupport.set(true);
   }

   public void initializeForSingleSupport()
   {
      isInDoubleSupport.set(false);
   }

   public void compute()
   {
      int numberOfFootstepsToConsider = this.numberOfFootstepsToConsider.getIntegerValue();

      if (numberOfFootstepsToConsider > numberOfFootstepsInPlan.getIntegerValue())
         numberOfFootstepsToConsider = numberOfFootstepsInPlan.getIntegerValue();
   }
}
