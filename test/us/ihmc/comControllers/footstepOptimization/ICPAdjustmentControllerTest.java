package us.ihmc.comControllers.footstepOptimization;

import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point2d;
import java.util.ArrayList;
import java.util.List;

public class ICPAdjustmentControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry("robert");

   private static final double footLengthForControl = 0.25;
   private static final double footWidthForControl = 0.12;
   private static final double toeWidthForControl = 0.12;

   private static final double initialTransferDuration = 2.0;
   private static final double singleSupportDuration = 1.0; /// 0.7;
   private static final double doubleSupportDuration = 0.2; // 0.4; //0.25;
   private static final double doubleSupportSplitFraction = 0.5;
   private static final boolean useTwoCMPs = true;

   private static final double maxDurationForSmoothingEntryToExitCMPSwitch = 1.0;
   private static final double timeSpentOnExitCMPInPercentOfStepTime = 0.5; // singleSupportDuration

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<>(new FramePose(), new FramePose());
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();

   private BipedSupportPolygons bipedSupportPolygons;

   private ICPAdjustmentController icpAdjustmentController;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void test()
   {
      setupController();
   }

   private void setupController()
   {
      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable yoTime = new DoubleYoVariable("t", registry);
      omega.set(3.0);

      setupContactableFeet();
      setupBipedSupportPolygons();

      int numberOfFootsteps = 6;
      double stepLength = 0.2;
      double stepWidth = 0.1;
      FootstepTestHelper footstepTestHelper = new FootstepTestHelper(contactableFeet, ankleFrames);
      List<Footstep> footsteps = footstepTestHelper.createFootsteps(stepWidth, stepLength, numberOfFootsteps);

      CapturePointPlannerParameters capturePointPlannerParameters = createCapturePointPlannerParameters();

      icpAdjustmentController = new ICPAdjustmentController(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, omega, registry);

      icpAdjustmentController.setDoubleSupportDuration(doubleSupportDuration);
      icpAdjustmentController.setSingleSupportDuration(doubleSupportDuration);

      icpAdjustmentController.clearPlan();
      for (int i = 0; i < numberOfFootsteps; i++)
         icpAdjustmentController.addFootstep(footsteps.get(i));

      icpAdjustmentController.initializeForSingleSupport(yoTime.getDoubleValue());
   }

   private void setupContactableFeet()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;

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
      }
   }

   private void setupBipedSupportPolygons()
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, null);
   }

   private CapturePointPlannerParameters createCapturePointPlannerParameters()
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
}
