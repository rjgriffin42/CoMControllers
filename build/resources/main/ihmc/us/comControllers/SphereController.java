package ihmc.us.comControllers;

import ihmc.us.comControllers.controllers.BasicHeightController;
import ihmc.us.comControllers.controllers.BasicPlanarController;
import ihmc.us.comControllers.controllers.SphereControlToolbox;
import ihmc.us.comControllers.model.SphereRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.RobotTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;

import javax.vecmath.Point2d;

public class SphereController implements RobotController
{
   private static final long serialVersionUID = -6115066729570319285L;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final BasicHeightController heightController;
   private final BasicPlanarController planarController;

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final FullRobotModel robot;
   private final SphereControlToolbox controlToolbox;
   private final ExternalForcePoint externalForcePoint;

   public SphereController(RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot, SphereControlToolbox controlToolbox, ExternalForcePoint externalForcePoint)
   {
      this.scsRobot = scsRobot;
      this.controlToolbox = controlToolbox;
      this.robot = controlToolbox.getFullRobotModel();
      this.externalForcePoint = externalForcePoint;

      heightController = new BasicHeightController(controlToolbox, registry);
      planarController = new BasicPlanarController(controlToolbox, registry);
   }

   private final Point2d planarForces = new Point2d();
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();
      robot.updateFrames();
      controlToolbox.update();

      heightController.doControl();
      planarController.doControl();
      planarController.getPlanarForces(planarForces);

      externalForcePoint.setForce(planarForces.getX(), planarForces.getY(), heightController.getVerticalForce());

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getDescription()
   {
      return registry.getName();
   }

   public String getName()
   {
      return registry.getName();
   }

}
