package us.ihmc.comControllers;

import us.ihmc.comControllers.controllers.SphereControlToolbox;
import us.ihmc.comControllers.model.SphereRobot;
import us.ihmc.comControllers.model.SphereRobotModel;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SphereSimulationTest
{
   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   public static void testPushRecovery() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException
   {
      Vector3D initialPosition = new Vector3D(0.0, 0.0, 1.0);
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


      //scs.createOverheadPlotter(scs, true, yoGraphicsListRegistry);

      scs.startOnAThread();

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);

      YoVariableRegistry rootRegistry = scs.getRootRegistry();

      YoBoolean sendFootsteps = (YoBoolean) rootRegistry.getVariable("sendFootsteps");
      YoDouble initialTransferDuration = (YoDouble) rootRegistry.getVariable("icpPlannerInitialTransferDuration");
      YoDouble singleSupportTime = (YoDouble) rootRegistry.getVariable("icpPlannerSingleSupportTime");
      YoDouble doubleSupportTime = (YoDouble) rootRegistry.getVariable("icpPlannerDoubleSupportTime");
      YoDouble controllerSingleSupportTime = (YoDouble) rootRegistry.getVariable("controllerSingleSupportDuration");
      YoDouble controllerDoubleSupportTime = (YoDouble) rootRegistry.getVariable("controllerDoubleSupportDuration");

      double stepTime = 1.0;
      double dsRatio = 0.15;
      double singleSupport = (1.0 - dsRatio) * stepTime;
      double doubleSupport = dsRatio * stepTime;
      singleSupportTime.set(singleSupport);
      doubleSupportTime.set(doubleSupport);
      controllerSingleSupportTime.set(singleSupport);
      controllerDoubleSupportTime.set(doubleSupport);

      double phaseInSSForPush = 0.7;

      double initializationTime = 3.0;

      double timeForPush = initializationTime + initialTransferDuration.getDoubleValue() + phaseInSSForPush * singleSupportTime.getDoubleValue();

      blockingSimulationRunner.simulateAndBlock(initializationTime);

      double timeIncrement = 0.1;
      double simulationDuration = 20.0;

      sendFootsteps.set(true);

      double timeAtStartOfPush = 0.0;
      boolean applyForce = true;

      double totalRobotMass = TotalMassCalculator.computeSubTreeMass(sphereRobotModel.getElevator());
      double totalRobotWeight = 9.81 * totalRobotMass;

      //Vector3D forceToApply = new Vector3D(0.0, -0.13 * totalRobotWeight, 0.0);
      Vector3D forceToApply = new Vector3D();
      Vector3D zero = new Vector3D(0.0, 0.0, 0.0);
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
