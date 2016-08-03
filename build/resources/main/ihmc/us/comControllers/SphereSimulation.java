package ihmc.us.comControllers;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;

import javax.vecmath.Vector3d;

public class SphereSimulation
{
   SimulationConstructionSet sim;

   public SphereSimulation()
   {
      Vector3d initialPosition = new Vector3d(0.0, 0.0, 1.0);
      SphereRobot sphereRobot = new SphereRobot("SphereRobot", initialPosition);

      SphereController controller = new SphereController(sphereRobot);
      sphereRobot.setController(controller);

      setupGroundContactModel(sphereRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      sim = new SimulationConstructionSet(sphereRobot, parameters);

      sim.setDT(0.001, 1);

      sim.setCameraPosition(-1.5, -2.5, 0.5);
      sim.setCameraFix(0.0, 0.0, 0.4);

      sim.setCameraTracking(false, true, true, false);
      sim.setCameraDolly(false, true, true, false);

      // Set up some graphs:

      /*
      sim.setupGraph("qd_x");
      sim.setupGraph("qd_y");
      sim.setupGraph("qd_z");

      sim.setupGraph("qd_wx");
      sim.setupGraph("qd_wy");
      sim.setupGraph("qd_wz");

      sim.setupEntryBox("qd_x");
      sim.setupEntryBox("qd_y");
      sim.setupEntryBox("qd_z");

      sim.setupEntryBox("qd_wx");
      sim.setupEntryBox("qd_wy");
      sim.setupEntryBox("qd_wz");
      */

      sim.startOnAThread();

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
