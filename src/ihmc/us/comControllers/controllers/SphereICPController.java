package ihmc.us.comControllers.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

import javax.vecmath.Vector3d;

public class SphereICPController implements GenericSphereController
{
   private enum SupportState {STANDING, DOUBLE, SINGLE}

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getName());

   private final BooleanYoVariable isInDoubleSupport = new BooleanYoVariable("isInDoubleSupport", registry);

   private final SphereControlToolbox controlToolbox;
   private final BasicHeightController heightController;

   private final StateMachine<SupportState> stateMachine;

   private final ICPPlanner icpPlanner;
   private final SideDependentList<YoPlaneContactState> contactStates;

   public SphereICPController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.controlToolbox = controlToolbox;

      isInDoubleSupport.set(true);
      contactStates = controlToolbox.getContactStates();

      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new ICPPlanner(controlToolbox.getBipedSupportPolygons(), controlToolbox.getContactableFeet(),
            controlToolbox.getCapturePointPlannerParameters(), registry, controlToolbox.getYoGraphicsListRegistry());
      icpPlanner.setDoubleSupportTime(controlToolbox.getDoubleSupportDuration());
      icpPlanner.setSingleSupportTime(controlToolbox.getSingleSupportDuration());
      icpPlanner.setOmega0(controlToolbox.getOmega0());
      icpPlanner.setDesiredCapturePointState(new FramePoint2d(ReferenceFrame.getWorldFrame()), new FrameVector2d(ReferenceFrame.getWorldFrame()));

      stateMachine = new StateMachine<>("supportStateMachine", "supportStateTime", SupportState.class, controlToolbox.getYoTime(), registry);
      StandingState standingState = new StandingState();
      DoubleSupportState doubleSupportState = new DoubleSupportState();
      SingleSupportState singleSupportState = new SingleSupportState();

      standingState.setDefaultNextState(doubleSupportState.getStateEnum());
      doubleSupportState.setDefaultNextState(singleSupportState.getStateEnum());
      singleSupportState.addStateTransition(new StateTransition<>(SupportState.DOUBLE, controlToolbox.getYoTime(),
            new TransitionToDoubleSupportCondition()));
      singleSupportState.addStateTransition(new StateTransition<>(SupportState.STANDING, controlToolbox.getYoTime(),
            new TransitionToStandingCondition()));

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   private final Vector3d forces = new Vector3d();
   public Vector3d getForces()
   {
      return forces;
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
         icpPlanner.initializeForStanding(controlToolbox.getYoTime().getDoubleValue());

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

      }

      @Override public void doTransitionIntoAction()
      {
         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();

         Footstep nextFootstep = controlToolbox.getFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(1);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         icpPlanner.addFootstepToPlan(nextFootstep);
         icpPlanner.addFootstepToPlan(nextNextFootstep);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep);

         RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setTransferFromSide(transferToSide);
         icpPlanner.initializeForTransfer(controlToolbox.getYoTime().getDoubleValue());
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private class TransitionToStandingCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return !controlToolbox.hasFootsteps();
      }
   }

   private class TransitionToDoubleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return controlToolbox.hasFootsteps();
      }
   }
}
