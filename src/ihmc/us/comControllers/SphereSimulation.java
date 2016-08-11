package ihmc.us.comControllers;

import ihmc.us.comControllers.controllers.SphereControlToolbox;
import ihmc.us.comControllers.model.SphereRobot;
import ihmc.us.comControllers.model.SphereRobotModel;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class SphereSimulation
{
   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   private final SimulationConstructionSet scs;

   public SphereSimulation()
   {
      Vector3d initialPosition = new Vector3d(0.0, 0.0, 1.0);
      SphereRobotModel sphereRobotModel = new SphereRobotModel();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      RobotTools.SCSRobotFromInverseDynamicsRobotModel sphereRobot = SphereRobot.createSphereRobot("SphereRobot", initialPosition,
            sphereRobotModel.getElevator(), yoGraphicsListRegistry, gravity);

      ExternalForcePoint externalForcePoint = sphereRobot.getAllExternalForcePoints().get(0);

      SphereControlToolbox sphereControlToolbox = new SphereControlToolbox(sphereRobotModel, controlDT, desiredHeight, gravity, sphereRobot.getYoTime(),
            sphereRobot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      SphereController controller = new SphereController(sphereRobot, sphereControlToolbox, externalForcePoint);
      sphereRobot.setController(controller);

      setupGroundContactModel(sphereRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      scs = new SimulationConstructionSet(sphereRobot, parameters);

      scs.setDT(controlDT, 1);

      scs.setCameraPosition(-1.5, -2.5, 0.5);
      scs.setCameraFix(0.0, 0.0, 0.4);

      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);

      VisualizerUtils.createOverheadPlotter(scs, true, yoGraphicsListRegistry);

      // Set up some graphs:

      /*
      scs.setupGraph("qd_x");
      scs.setupGraph("qd_y");
      scs.setupGraph("qd_z");

      scs.setupGraph("qd_wx");
      scs.setupGraph("qd_wy");
      scs.setupGraph("qd_wz");

      scs.setupEntryBox("qd_x");
      scs.setupEntryBox("qd_y");
      scs.setupEntryBox("qd_z");

      scs.setupEntryBox("qd_wx");
      scs.setupEntryBox("qd_wy");
      scs.setupEntryBox("qd_wz");
      */

      scs.startOnAThread();

   }

   private void setupGroundContactModel(Robot robot)
   {
      double kXY = 1000.0; //1422.0;
      double bXY = 100.0; //150.6;
      double kZ = 20.0; //50.0;
      double bZ = 50.0; //1000.0;
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry());

      double amplitude = 0.1;
      double frequency = 0.3;
      double offset = 0.5;
      GroundProfile3D groundProfile = new RollingGroundProfile(amplitude, frequency, offset);
      groundContactModel.setGroundProfile3D(groundProfile);
      robot.setGroundContactModel(groundContactModel);

   }

   public static void main(String[] args)
   {
      new SphereSimulation();
   }
}
