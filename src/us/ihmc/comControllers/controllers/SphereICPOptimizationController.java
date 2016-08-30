package us.ihmc.comControllers.controllers;

import us.ihmc.comControllers.icpOptimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPProportionalController;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class SphereICPOptimizationController implements GenericSphereController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum SupportState {STANDING, DOUBLE, SINGLE}

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d planarForces = new YoFramePoint2d("planarICPForces", worldFrame, registry);
   private final YoFrameVector desiredForces = new YoFrameVector("desiredForces", worldFrame, registry);
   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);

   private final BagOfBalls cmpTrack;
   private final BagOfBalls icpTrack;
   private final BagOfBalls comTrack;

   private final SphereControlToolbox controlToolbox;
   private final BasicHeightController heightController;

   private final StateMachine<SupportState> stateMachine;

   private final ICPPlanner icpPlanner;

   private final ReferenceFrame centerOfMassFrame;

   private final SideDependentList<YoPlaneContactState> contactStates;
   private final SideDependentList<FootSpoof> contactableFeet;
   private final SideDependentList<FramePose> footPosesAtTouchdown;

   private final YoFramePoint icp;
   private final YoFramePoint desiredICP;
   private final YoFrameVector desiredICPVelocity;

   private final YoFramePoint yoDesiredCMP;

   private final ICPOptimizationController icpOptimizationController;
   private final ICPProportionalController icpController;
   private final ICPControlGains icpGains;
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);
   private final double totalMass;

   private final int numberOfBalls = 100;
   private final int simulatedTicksPerGraphicUpdate = 16;


   private final DoubleYoVariable yoTime;

   public SphereICPOptimizationController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.controlToolbox = controlToolbox;

      isInDoubleSupport.set(true);
      contactStates = controlToolbox.getContactStates();
      contactableFeet = controlToolbox.getContactableFeet();
      yoTime = controlToolbox.getYoTime();
      footPosesAtTouchdown = controlToolbox.getFootPosesAtTouchdown();
      centerOfMassFrame = controlToolbox.getCenterOfMassFrame();
      totalMass = TotalMassCalculator.computeSubTreeMass(controlToolbox.getFullRobotModel().getElevator());

      icp = controlToolbox.getICP();
      desiredICP = controlToolbox.getDesiredICP();
      desiredICPVelocity = controlToolbox.getDesiredICPVelocity();

      yoDesiredCMP = controlToolbox.getDesiredCMP();

      YoGraphicsListRegistry yoGraphicsListRegistry = controlToolbox.getYoGraphicsListRegistry();

      omega0.set(controlToolbox.getOmega0());
      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new ICPPlanner(controlToolbox.getBipedSupportPolygons(), controlToolbox.getContactableFeet(),
            controlToolbox.getCapturePointPlannerParameters(), registry, yoGraphicsListRegistry);
      icpPlanner.setDoubleSupportTime(controlToolbox.getDoubleSupportDuration());
      icpPlanner.setSingleSupportTime(controlToolbox.getSingleSupportDuration());
      icpPlanner.setOmega0(omega0.getDoubleValue());
      icpPlanner.setDesiredCapturePointState(new FramePoint2d(ReferenceFrame.getWorldFrame()), new FrameVector2d(ReferenceFrame.getWorldFrame()));

      icpGains = new ICPControlGains("CoMController", registry);
      icpGains.setKpOrthogonalToMotion(3.0);
      icpGains.setKpParallelToMotion(2.0);

      icpController = new ICPProportionalController(icpGains, controlToolbox.getControlDT(), registry);
      icpOptimizationController = new ICPOptimizationController(controlToolbox.getCapturePointPlannerParameters(), controlToolbox.getICPOptimizationParameters(),
            controlToolbox.getBipedSupportPolygons(), controlToolbox.getContactableFeet(), omega0, registry);
      icpOptimizationController.setStepDurations(controlToolbox.getDoubleSupportDuration(), controlToolbox.getSingleSupportDuration());

      stateMachine = new StateMachine<>("supportStateMachine", "supportStateTime", SupportState.class, controlToolbox.getYoTime(), registry);
      StandingState standingState = new StandingState();
      DoubleSupportState doubleSupportState = new DoubleSupportState();
      SingleSupportState singleSupportState = new SingleSupportState();

      standingState.setDefaultNextState(doubleSupportState.getStateEnum());
      doubleSupportState.setDefaultNextState(singleSupportState.getStateEnum());

      singleSupportState.addStateTransition(new StateTransition<>(doubleSupportState.getStateEnum(), new TransitionToDoubleSupportCondition()));
      singleSupportState.addStateTransition(new StateTransition<>(standingState.getStateEnum(), new TransitionToStandingCondition()));

      stateMachine.addState(standingState);
      stateMachine.addState(doubleSupportState);
      stateMachine.addState(singleSupportState);
      stateMachine.setCurrentState(SupportState.STANDING);

      cmpTrack = new BagOfBalls(numberOfBalls, 0.01, "eCMP", YoAppearance.Purple(), registry, yoGraphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfBalls, 0.01, "ICP", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      comTrack = new BagOfBalls(numberOfBalls, 0.01, "CoM", YoAppearance.Black(), registry, yoGraphicsListRegistry);

      YoGraphicVector forceVisualizer = new YoGraphicVector("forceViz", yoDesiredCMP, desiredForces, 0.05, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("forceViz", forceVisualizer);

      parentRegistry.addChild(registry);
   }

   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint desiredCapturePoint = new FramePoint();
   private final FramePoint finalDesiredCapturePoint = new FramePoint();
   private final FrameVector desiredCapturePointVelocity = new FrameVector();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();

   private int counter = 0;
   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      heightController.doControl();

      controlToolbox.update();

      icp.getFrameTuple2d(capturePoint2d);
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePoint, desiredCapturePointVelocity, yoTime.getDoubleValue());
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePoint);
      desiredICP.set(desiredCapturePoint);
      desiredCapturePointVelocity.set(desiredCapturePointVelocity);

      desiredCapturePoint2d.setByProjectionOntoXYPlane(desiredCapturePoint);
      desiredCapturePointVelocity2d.setByProjectionOntoXYPlane(desiredCapturePointVelocity);
      finalDesiredCapturePoint2d.setByProjectionOntoXYPlane(finalDesiredCapturePoint);

      FramePoint2d desiredCMP = yoDesiredCMP.getFramePoint2dCopy();

      double fZ = heightController.getVerticalForce();
      FrameVector reactionForces = computeGroundReactionForce(desiredCMP, fZ);
      reactionForces.changeFrame(worldFrame);
      planarForces.setByProjectionOntoXYPlane(reactionForces);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         icpTrack.setBallLoop(desiredICP.getFramePointCopy());
         cmpTrack.setBallLoop(yoDesiredCMP.getFramePointCopy());
         comTrack.setBallLoop(centerOfMass);
      }
   }

   public Vector3d getForces()
   {
      desiredForces.setX(planarForces.getX());
      desiredForces.setY(planarForces.getY());
      desiredForces.setZ(heightController.getVerticalForce());

      return desiredForces.getVector3dCopy();
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();
   private final FramePoint centerOfMass = new FramePoint();
   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0.getDoubleValue());

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   private class StandingState extends State<SupportState>
   {
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public StandingState()
      {
         super(SupportState.STANDING);
      }

      @Override public void doAction()
      {
         if (controlToolbox.hasFootsteps())
            this.transitionToDefaultNextState();

         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, capturePoint2d);
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.setXY(desiredCMP);
      }

      @Override public void doTransitionIntoAction()
      {
         desiredICP.getFrameTuple(desiredCapturePoint);

         icpPlanner.clearPlan();
         icpPlanner.holdCurrentICP(yoTime.getDoubleValue(), desiredCapturePoint);
         icpPlanner.initializeForStanding(yoTime.getDoubleValue());
         icpPlanner.setDesiredCapturePointState(desiredICP, desiredICPVelocity);

         icpPlanner.clearPlan();
         icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private class SingleSupportState extends State<SupportState>
   {
      private final FramePoint2d desiredCMP = new FramePoint2d();
      public SingleSupportState()
      {
         super(SupportState.SINGLE);
      }

      @Override public void doAction()
      {
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, capturePoint2d);
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.setXY(desiredCMP);
         /*
         FramePoint2d desiredCMP = icpController.doProportionalControl(null, capturePoint2d, desiredCapturePoint2d, finalDesiredCapturePoint2d,
               desiredCapturePointVelocity2d, null, omega0.getDoubleValue());
         yoDesiredCMP.setXY(desiredCMP);
               */
      }

      @Override public void doTransitionIntoAction()
      {
         icpPlanner.clearPlan();
         icpOptimizationController.clearPlan();

         Footstep nextFootstep = controlToolbox.getFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(1);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         icpPlanner.addFootstepToPlan(nextFootstep);
         icpPlanner.addFootstepToPlan(nextNextFootstep);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep);

         icpOptimizationController.addFootstepToPlan(nextFootstep);
         icpOptimizationController.addFootstepToPlan(nextNextFootstep);
         icpOptimizationController.addFootstepToPlan(nextNextNextFootstep);

         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
         icpPlanner.setSupportLeg(supportSide);
         icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

         icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide);

         FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
         FramePose nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
         nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
         nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
         footSpoof.setSoleFrame(nextSupportPose);

         contactStates.get(supportSide.getOppositeSide()).clear();
         if (nextFootstep.getPredictedContactPoints() == null)
            contactStates.get(supportSide.getOppositeSide()).setContactFramePoints(footSpoof.getContactPoints2d());
         else
            contactStates.get(supportSide.getOppositeSide()).setContactPoints(nextFootstep.getPredictedContactPoints());
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private class DoubleSupportState extends State<SupportState>
   {
      private final FramePoint2d desiredCMP = new FramePoint2d();
      public DoubleSupportState()
      {
         super(SupportState.DOUBLE);
      }

      @Override public void doAction()
      {
         if (icpPlanner.isDone(yoTime.getDoubleValue()))
            transitionToDefaultNextState();

         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, capturePoint2d);
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.setXY(desiredCMP);
         /*
         FramePoint2d desiredCMP = icpController.doProportionalControl(null, capturePoint2d, desiredCapturePoint2d, finalDesiredCapturePoint2d,
               desiredCapturePointVelocity2d, null, omega0.getDoubleValue());
         yoDesiredCMP.setXY(desiredCMP);
         */
      }

      @Override public void doTransitionIntoAction()
      {
         icpPlanner.clearPlan();
         icpOptimizationController.clearPlan();

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
         controlToolbox.getBipedSupportPolygons().updateUsingContactStates(contactStates);

         Footstep nextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(1);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(2);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         icpPlanner.addFootstepToPlan(nextFootstep);
         icpPlanner.addFootstepToPlan(nextNextFootstep);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep);

         icpOptimizationController.addFootstepToPlan(nextFootstep);
         icpOptimizationController.addFootstepToPlan(nextNextFootstep);
         icpOptimizationController.addFootstepToPlan(nextNextNextFootstep);

         RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setTransferToSide(transferToSide);
         icpPlanner.initializeForTransfer(yoTime.getDoubleValue());

         icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide);
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private class TransitionToStandingCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         if (icpPlanner.isDone(yoTime.getDoubleValue()))
            return !controlToolbox.hasFootsteps();

         return false;
      }
   }

   private class TransitionToDoubleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         if (icpPlanner.isDone(yoTime.getDoubleValue()))
            return controlToolbox.hasFootsteps();

         return false;
      }
   }
}
