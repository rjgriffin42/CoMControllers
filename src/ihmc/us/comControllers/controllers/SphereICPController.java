package ihmc.us.comControllers.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPProportionalController;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class SphereICPController implements GenericSphereController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum SupportState {STANDING, DOUBLE, SINGLE}

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d planarForces = new YoFramePoint2d("planarICPForces", worldFrame, registry);
   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);

   private final BagOfBalls cmpTrack;
   private final BagOfBalls icpTrack;

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

   private final YoFramePoint2d yoDesiredCMP;

   private final ICPProportionalController icpController;
   private final ICPControlGains icpGains;
   private final double omega0;
   private final double totalMass;

   private final int numberOfBalls = 100;
   private final int simulatedTicksPerGraphicUpdate = 16;


   private final DoubleYoVariable yoTime;

   public SphereICPController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
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

      omega0 = controlToolbox.getOmega0();
      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new ICPPlanner(controlToolbox.getBipedSupportPolygons(), controlToolbox.getContactableFeet(),
            controlToolbox.getCapturePointPlannerParameters(), registry, yoGraphicsListRegistry);
      icpPlanner.setDoubleSupportTime(controlToolbox.getDoubleSupportDuration());
      icpPlanner.setSingleSupportTime(controlToolbox.getSingleSupportDuration());
      icpPlanner.setOmega0(omega0);
      icpPlanner.setDesiredCapturePointState(new FramePoint2d(ReferenceFrame.getWorldFrame()), new FrameVector2d(ReferenceFrame.getWorldFrame()));

      icpGains = new ICPControlGains("CoMController", registry);
      icpGains.setKpOrthogonalToMotion(3.0);
      icpGains.setKpParallelToMotion(2.0);

      icpController = new ICPProportionalController(icpGains, controlToolbox.getControlDT(), registry);

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

      parentRegistry.addChild(registry);
   }

   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint desiredCapturePoint = new FramePoint();
   private final FramePoint finalDesiredCapturePoint = new FramePoint();
   private final FrameVector desiredCapturePointVelocity = new FrameVector();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();
   private final FramePoint cmp = new FramePoint();

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

      FramePoint2d desiredCMP = icpController.doProportionalControl(null, capturePoint2d, desiredCapturePoint2d, finalDesiredCapturePoint2d,
            desiredCapturePointVelocity2d, null, omega0);

      double fZ = heightController.getVerticalForce();
      FrameVector reactionForces = computeGroundReactionForce(desiredCMP, fZ);
      reactionForces.changeFrame(worldFrame);
      planarForces.setByProjectionOntoXYPlane(reactionForces);

      desiredCMP.changeFrame(worldFrame);
      yoDesiredCMP.set(desiredCMP);
      cmp.setXY(desiredCMP);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         icpTrack.setBallLoop(desiredICP.getFramePointCopy());
         cmpTrack.setBallLoop(cmp);
      }
   }

   private final Vector3d forces = new Vector3d();
   public Vector3d getForces()
   {
      forces.setX(planarForces.getX());
      forces.setY(planarForces.getY());
      forces.setZ(heightController.getVerticalForce());

      return forces;
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();
   private final FramePoint centerOfMass = new FramePoint();
   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   private class StandingState extends State<SupportState>
   {
      public StandingState()
      {
         super(SupportState.STANDING);
      }

      @Override public void doAction()
      {
         if (controlToolbox.hasFootsteps())
            this.transitionToDefaultNextState();
      }

      @Override public void doTransitionIntoAction()
      {
         desiredICP.getFrameTuple(desiredCapturePoint);

         icpPlanner.clearPlan();
         icpPlanner.holdCurrentICP(yoTime.getDoubleValue(), desiredCapturePoint);
         icpPlanner.initializeForStanding(yoTime.getDoubleValue());
         icpPlanner.setDesiredCapturePointState(desiredICP, desiredICPVelocity);

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private class SingleSupportState extends State<SupportState>
   {
      public SingleSupportState()
      {
         super(SupportState.SINGLE);
      }

      @Override public void doAction()
      {
      }

      @Override public void doTransitionIntoAction()
      {
         icpPlanner.clearPlan();

         Footstep nextFootstep = controlToolbox.getFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(1);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         icpPlanner.addFootstepToPlan(nextFootstep);
         icpPlanner.addFootstepToPlan(nextNextFootstep);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep);

         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
         icpPlanner.setSupportLeg(supportSide);
         icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());


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
      public DoubleSupportState()
      {
         super(SupportState.DOUBLE);
      }

      @Override public void doAction()
      {
         if (icpPlanner.isDone(yoTime.getDoubleValue()))
            transitionToDefaultNextState();
      }

      @Override public void doTransitionIntoAction()
      {
         icpPlanner.clearPlan();

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

         RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setTransferToSide(transferToSide);
         icpPlanner.initializeForTransfer(yoTime.getDoubleValue());
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
