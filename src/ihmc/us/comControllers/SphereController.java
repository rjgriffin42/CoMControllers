package ihmc.us.comControllers;

import ihmc.us.comControllers.controllers.BasicHeightController;
import ihmc.us.comControllers.model.SphereRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.RobotTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class SphereController implements RobotController
{
   private static final long serialVersionUID = -6115066729570319285L;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final BasicHeightController heightController;

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SphereRobotModel robot;
   private final ExternalForcePoint externalForcePoint;

   public SphereController(RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot, SphereRobotModel robot, ExternalForcePoint externalForcePoint, double controlDT)
   {
      this.scsRobot = scsRobot;
      this.robot = robot;
      this.externalForcePoint = externalForcePoint;

      heightController = new BasicHeightController(robot, controlDT, registry);
   }
   

   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();
      robot.update();

      heightController.doControl();

      externalForcePoint.setForce(0.0, 0.0, heightController.getVerticalForce());

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
