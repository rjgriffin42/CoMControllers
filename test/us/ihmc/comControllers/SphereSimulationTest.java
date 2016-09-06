package us.ihmc.comControllers;

import org.junit.Test;
import us.ihmc.comControllers.controllers.SphereControlToolbox;
import us.ihmc.comControllers.model.SphereRobot;
import us.ihmc.comControllers.model.SphereRobotModel;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class SphereSimulationTest
{
   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   public static void testPushRecovery() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException
   {
      Vector3d initialPosition = new Vector3d(0.0, 0.0, 1.0);
      SphereRobotModel sphereRobotModel = new SphereRobotModel();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      RobotTools.SCSRobotFromInverseDynamicsRobotModel sphereRobot = SphereRobot.createSphereRobot("SphereRobot", initialPosition,
            sphereRobotModel.getElevator(), yoGraphicsListRegistry, gravity);

      ExternalForcePoint externalForcePoint = sphereRobot.getAllExternalForcePoints().get(0);
      ExternalForcePoint pushPoint = new ExternalForcePoint("pushPoint", sphereRobot.getRobotsYoVariableRegistry());
      sphereRobot.getRootJoints().get(0).addExternalForcePoint(pushPoint);

      YoGraphicVector externalForceGraphic = new YoGraphicVector(pushPoint.getName() + "Force", pushPoint.getYoPosition(),
            pushPoint.getYoForce(), 0.1);
      yoGraphicsListRegistry.registerYoGraphic("ExternalForces", externalForceGraphic);

      SphereControlToolbox sphereControlToolbox = new SphereControlToolbox(sphereRobotModel, controlDT, desiredHeight, gravity, sphereRobot.getYoTime(),
            sphereRobot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      SphereController controller = new SphereController(sphereRobot, sphereControlToolbox, externalForcePoint);
      sphereRobot.setController(controller);

      setupGroundContactModel(sphereRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(sphereRobot, parameters);

      scs.setDT(controlDT, 1);

      scs.setCameraPosition(-1.5, -2.5, 0.5);
      scs.setCameraFix(0.0, 0.0, 0.4);

      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);

      VisualizerUtils.createOverheadPlotter(scs, true, yoGraphicsListRegistry);

      scs.startOnAThread();

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);

      YoVariableRegistry rootRegistry = scs.getRootRegistry();

      BooleanYoVariable sendFootsteps = (BooleanYoVariable) rootRegistry.getVariable("sendFootsteps");
      DoubleYoVariable initialTransferDuration = (DoubleYoVariable) rootRegistry.getVariable("icpPlannerInitialTransferDuration");
      DoubleYoVariable singleSupportTime = (DoubleYoVariable) rootRegistry.getVariable("icpPlannerSingleSupportTime");

      double initializationTime = 3.0;
      double timeForPush = initializationTime + initialTransferDuration.getDoubleValue() + 0.7 * singleSupportTime.getDoubleValue();

      blockingSimulationRunner.simulateAndBlock(initializationTime);

      double timeIncrement = 0.1;
      double simulationDuration = 20.0;

      sendFootsteps.set(true);

      double timeAtStartOfPush = 0.0;
      boolean applyForce = true;

      double totalRobotMass = TotalMassCalculator.computeSubTreeMass(sphereRobotModel.getElevator());
      double totalRobotWeight = 9.81 * totalRobotMass;

      //Vector3d forceToApply = new Vector3d(0.0, -0.13 * totalRobotWeight, 0.0);
      Vector3d forceToApply = new Vector3d();
      Vector3d zero = new Vector3d(0.0, 0.0, 0.0);
      double pushDuration = 0.2;

      while (scs.getTime() - initializationTime < simulationDuration)
      {
         double time = scs.getTime();
         if (time > timeForPush && applyForce)
         {
            timeAtStartOfPush = time;
            applyForce = false;
         }

         if ((timeAtStartOfPush > 0.0) && time < (timeAtStartOfPush + pushDuration))
         {
            pushPoint.setForce(forceToApply);
         }
         else
         {
            pushPoint.setForce(zero);
         }

         blockingSimulationRunner.simulateAndBlock(timeIncrement);
      }
   }

   private static void setupGroundContactModel(Robot robot)
   {
      double kXY = 1000.0; //1422.0;
      double bXY = 100.0; //150.6;
      double kZ = 20.0; //50.0;
      double bZ = 50.0; //1000.0;
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry());

      GroundProfile3D groundProfile = new FlatGroundProfile();
      groundContactModel.setGroundProfile3D(groundProfile);
      robot.setGroundContactModel(groundContactModel);
   }

   public static void main(String[] args)
   {
      try
      {
         testPushRecovery();
      }
      catch (BlockingSimulationRunner.SimulationExceededMaximumTimeException e)
      {
      }
      catch (ControllerFailureException e)
      {
      }
   }
}
