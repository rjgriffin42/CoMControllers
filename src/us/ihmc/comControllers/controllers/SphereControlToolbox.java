package us.ihmc.comControllers.controllers;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.comControllers.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.*;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicShape;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

import javax.vecmath.Point2d;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class SphereControlToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double footLengthForControl = 0.25;
   private static final double footWidthForControl = 0.12;
   private static final double toeWidthForControl = 0.12;

   private static final double initialTransferDuration = 2.0;
   private static final double singleSupportDuration = 2.0;
   private static final double doubleSupportDuration = 1.0; // 0.4; //0.25;
   private static final double doubleSupportSplitFraction = 0.5;
   private static final boolean useTwoCMPs = true;

   private static final double maxDurationForSmoothingEntryToExitCMPSwitch = 1.0;
   private static final double timeSpentOnExitCMPInPercentOfStepTime = 0.5; // singleSupportDuration

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint eCMP = new YoFramePoint("eCMP", worldFrame, registry);
   private final YoFramePoint desiredICP = new YoFramePoint("desiredICP", worldFrame, registry);
   private final YoFrameVector desiredICPVelocity = new YoFrameVector("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint desiredCMP = new YoFramePoint("desiredCMP", worldFrame, registry);

   private final YoFramePoint icp = new YoFramePoint("icp", worldFrame, registry);

   private final YoFramePoint yoCenterOfMass = new YoFramePoint("centerOfMass", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocity = new YoFrameVector("centerOfMassVelocity", worldFrame, registry);
   private final YoFramePoint2d yoCenterOfMass2d = new YoFramePoint2d("centerOfMass2d", worldFrame, registry);
   private final YoFrameVector2d yoCenterOfMassVelocity2d = new YoFrameVector2d("centerOfMassVelocity2d", worldFrame, registry);

   private final BooleanYoVariable sendFootsteps = new BooleanYoVariable("sendFootsteps", registry);
   private final BooleanYoVariable hasFootsteps = new BooleanYoVariable("hasFootsteps", registry);
   private final IntegerYoVariable numberOfFootsteps = new IntegerYoVariable("numberOfFootsteps", registry);

   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final RigidBody elevator;
   private final FullRobotModel fullRobotModel;

   private final TwistCalculator twistCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<FramePose>(new FramePose(), new FramePose());
   private final SideDependentList<YoFramePose> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private BipedSupportPolygons bipedSupportPolygons;

   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final ArrayList<Footstep> footsteps = new ArrayList<>();

   private final YoFramePose yoNextFootstepPose = new YoFramePose("nextFootstepPose", worldFrame, registry);
   private final YoFramePose yoNextNextFootstepPose = new YoFramePose("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePose yoNextNextNextFootstepPose = new YoFramePose("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoNextFootstepPolygon = new YoFrameConvexPolygon2d("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2d yoNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2d yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final double controlDT;
   private final double desiredHeight;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private ReferenceFrame midFeetZUpFrame;

   private FootstepTestHelper footstepTestHelper;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private CapturePointPlannerParameters capturePointPlannerParameters;
   private ICPOptimizationParameters icpOptimizationParameters;

   private DoubleYoVariable yoTime;

   public SphereControlToolbox(FullRobotModel sphereRobotModel, double controlDT, double desiredHeight, double gravity, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = sphereRobotModel;
      this.controlDT = controlDT;
      this.desiredHeight = desiredHeight;
      this.yoTime = yoTime;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      elevator = sphereRobotModel.getElevator();

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

      setupFeetFrames(yoGraphicsListRegistry);

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", desiredCMP, 0.012, YoAppearance.Purple(), GraphicType.CROSS);
      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired Capture Point", desiredICP, 0.01, YoAppearance.Yellow(), GraphicType.ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("Capture Point", icp, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), GraphicType.ROTATED_CROSS);

      YoArtifactPolygon nextFootstepArtifact =  new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextFootstepArtifact =  new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextNextFootstepArtifact =  new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCMPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, comViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact(graphicListName, nextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact(graphicListName, nextNextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact(graphicListName, nextNextNextFootstepArtifact);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2d> contactPoints = new ArrayList<>();
      for (FramePoint2d point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(point.getPointCopy());
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));

      YoGraphicShape nextFootstepViz = new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0);
      YoGraphicShape nextNextFootstepViz = new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0);
      YoGraphicShape nextNextNextFootstepViz = new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0);

      yoGraphicsListRegistry.registerYoGraphic(graphicListName, nextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, nextNextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, nextNextNextFootstepViz);

      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               soleFrames.get(robotSide).update();
               ankleZUpFrames.get(robotSide).update();
            }
            midFeetZUpFrame.update();
         }
      });

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      twistCalculator = new TwistCalculator(worldFrame, sphereRobotModel.getRootJoint().getSuccessor());
      centerOfMassJacobian = new CenterOfMassJacobian(elevator);

      capturePointPlannerParameters = createICPPlannerParameters();
      icpOptimizationParameters = createICPOptimizationParameters();

      parentRegistry.addChild(registry);
   }

   private void setupFeetFrames(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2d> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2d(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2d(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2d(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2d(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         currentFootPoses.put(robotSide, new YoFramePose(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet, ankleFrames);

   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public double getDesiredHeight()
   {
      return desiredHeight;
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public YoFramePoint getCenterOfMass()
   {
      return yoCenterOfMass;
   }

   public YoFrameVector getCenterOfMassVelocity()
   {
      return yoCenterOfMassVelocity;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public SideDependentList<FootSpoof> getContactableFeet()
   {
      return contactableFeet;
   }

   public SideDependentList<YoPlaneContactState> getContactStates()
   {
      return contactStates;
   }

   public SideDependentList<FramePose> getFootPosesAtTouchdown()
   {
      return footPosesAtTouchdown;
   }

   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return icpOptimizationParameters;
   }

   public double getDoubleSupportDuration()
   {
      return doubleSupportDuration;
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration;
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public YoFramePoint getDesiredCMP()
   {
      return desiredCMP;
   }

   public YoFramePoint getDesiredICP()
   {
      return desiredICP;
   }

   public YoFrameVector getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public YoFramePoint getICP()
   {
      return icp;
   }

   public FramePoint2d getCapturePoint2d()
   {
      return capturePoint2d;
   }

   public void update()
   {
      centerOfMassFrame.update();

      twistCalculator.compute();
      centerOfMassJacobian.compute();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      callUpdatables();
      updateFootViz();
      computeCenterOfMass();
      computeCapturePoint();

      if (sendFootsteps.getBooleanValue())
      {
         for (Footstep footstep : footstepTestHelper.createFootsteps(0.25, 0.2, 100))
            footsteps.add(footstep);

         sendFootsteps.set(false);
      }

      hasFootsteps.set(footsteps.size() > 0);
      numberOfFootsteps.set(footsteps.size());
   }

   private void updateFootViz()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (contactStates.get(robotSide).inContact())
         {
            FramePose footPose = new FramePose(contactableFeet.get(robotSide).getSoleFrame());
            footPose.changeFrame(worldFrame);
            currentFootPoses.get(robotSide).set(footPose);
         }
         else
         {
            currentFootPoses.get(robotSide).setToNaN();
         }
      }
   }

   public boolean hasFootsteps()
   {
      return hasFootsteps.getBooleanValue();
   }

   public int getNumberOfSteps()
   {
      return numberOfFootsteps.getIntegerValue();
   }

   public Footstep getFootstep(int i)
   {
      return footsteps.remove(i);
   }

   public Footstep peekAtFootstep(int i)
   {
      if (i >= footsteps.size())
         return null;
      else
         return footsteps.get(i);
   }

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   public void updateUpcomingFootstepsViz(Footstep nextFootstep, Footstep nextNextFootstep, Footstep nextNextNextFootstep)
   {
      if (nextFootstep == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.hide();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextFootstep.getPredictedContactPoints() == null)
         nextFootstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(nextFootstep.getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextFootstep.getSoleReferenceFrame(), nextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextFootstepPose = new FramePose(nextFootstep.getSoleReferenceFrame());
      yoNextFootstepPose.setAndMatchFrame(nextFootstepPose);

      if (nextNextFootstep == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextNextFootstep.getPredictedContactPoints() == null)
         nextNextFootstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(nextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextNextFootstep.getSoleReferenceFrame(), nextNextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextFootstepPose = new FramePose(nextNextFootstep.getSoleReferenceFrame());
      yoNextNextFootstepPose.setAndMatchFrame(nextNextFootstepPose);

      if (nextNextNextFootstep == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextNextNextFootstep.getPredictedContactPoints() == null)
         nextNextNextFootstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(nextNextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextNextNextFootstep.getSoleReferenceFrame(), nextNextNextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextNextFootstepPose = new FramePose(nextNextNextFootstep.getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setAndMatchFrame(nextNextNextFootstepPose);
   }

   private void callUpdatables()
   {
      for (Updatable updatable : updatables)
         updatable.update(yoTime.getDoubleValue());
   }


   private final FramePoint centerOfMass = new FramePoint();
   private final FrameVector centerOfMassVelocity = new FrameVector();
   private final FramePoint2d centerOfMass2d = new FramePoint2d();
   private final FrameVector2d centerOfMassVelocity2d = new FrameVector2d();
   public void computeCenterOfMass()
   {
      centerOfMass.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMass2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMass);
      centerOfMassVelocity2d.setByProjectionOntoXYPlaneIncludingFrame(centerOfMassVelocity);

      yoCenterOfMass.set(centerOfMass);
      yoCenterOfMassVelocity.set(centerOfMassVelocity);

      yoCenterOfMass2d.set(centerOfMass2d);
      yoCenterOfMassVelocity2d.set(centerOfMassVelocity2d);
   }

   private final FramePoint2d capturePoint2d = new FramePoint2d();

   public void computeCapturePoint()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);

      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMass2d, centerOfMassVelocity2d, omega0.getDoubleValue());

      capturePoint2d.changeFrame(icp.getReferenceFrame());
      icp.setXY(capturePoint2d);
   }

   private CapturePointPlannerParameters createICPPlannerParameters()
   {
      return new CapturePointPlannerParameters()
      {

         @Override
         public double getDoubleSupportInitialTransferDuration()
         {
            return initialTransferDuration;
         }

         @Override
         public double getDoubleSupportSplitFraction()
         {
            return doubleSupportSplitFraction;
         }

         @Override
         public double getEntryCMPInsideOffset()
         {
            return -0.005; // 0.006;
         }

         @Override
         public double getExitCMPInsideOffset()
         {
            return 0.025;
         }

         @Override
         public double getEntryCMPForwardOffset()
         {
            return 0.0;
         }

         @Override
         public double getExitCMPForwardOffset()
         {
            return 0.0;
         }

         @Override
         public boolean useTwoCMPsPerSupport()
         {
            return useTwoCMPs;
         }

         @Override
         public double getTimeSpentOnExitCMPInPercentOfStepTime()
         {
            return timeSpentOnExitCMPInPercentOfStepTime;
         }

         @Override
         public double getMaxEntryCMPForwardOffset()
         {
            return 0.03;
         }

         @Override
         public double getMinEntryCMPForwardOffset()
         {
            return -0.05;
         }

         @Override
         public double getMaxExitCMPForwardOffset()
         {
            return 0.15;
         }

         @Override
         public double getMinExitCMPForwardOffset()
         {
            return -0.04;
         }

         @Override
         public double getCMPSafeDistanceAwayFromSupportEdges()
         {
            return 0.001;
         }

         @Override
         public double getMaxDurationForSmoothingEntryToExitCMPSwitch()
         {
            return maxDurationForSmoothingEntryToExitCMPSwitch;
         }

         /** {@inheritDoc} */
         @Override
         public boolean useExitCMPOnToesForSteppingDown()
         {
            return true;
         }
      };
   }

   public ICPOptimizationParameters createICPOptimizationParameters()
   {
      return new ICPOptimizationParameters()
      {
         @Override public int getMaximumNumberOfFootstepsToConsider()
         {
            return 5;
         }

         @Override
         public int numberOfFootstepsToConsider()
         {
            return 2;
         }

         @Override public double getFootstepWeight()
         {
            return 2.0;
         }

         @Override public double getFootstepRegularizationWeight()
         {
            return 0.01;
         }

         @Override public double getFeedbackWeight()
         {
            return 1.5;
         }

         @Override public double getFeedbackRegularizationWeight()
         {
            return 0.0001;
         }

         @Override public double getFeedbackGain()
         {
            return 4.0;
         }

         @Override public boolean useFeedback()
         {
            return true;
         }

         @Override public boolean useFeedbackRegularization()
         {
            return false;
         }

         @Override public boolean useStepAdjustment()
         {
            return true;
         }

         @Override public boolean useFootstepRegularization()
         {
            return false;
         }

         @Override public boolean useFeedbackWeightHardening()
         {
            return true;
         }

         @Override public boolean scaleFirstStepWeightWithTime()
         {
            return false;
         }

         @Override public boolean scaleUpcomingStepWeights()
         {
            return true;
         }

         @Override public boolean scaleFeedbackWeightWithGain()
         {
            return false;
         }

         @Override public double getMinimumFootstepWeight()
         {
            return 0.0001;
         }

         @Override public double getMinimumFeedbackWeight()
         {
            return 0.0001;
         }

         @Override public double getMinimumTimeRemaining()
         {
            return 0.001;
         }

         @Override public double getFeedbackWeightHardeningMultiplier()
         {
            return 20.0;
         }

         @Override public double getMaxCMPExitForward()
         {
            return 0.05;
         }

         @Override public double getMaxCMPExitSideways()
         {
            return 0.03;
         }
      };
   }
}
