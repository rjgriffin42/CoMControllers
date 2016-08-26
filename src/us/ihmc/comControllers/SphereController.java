package us.ihmc.comControllers;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.comControllers.controllers.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.RobotTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class SphereController implements RobotController
{
   private enum SphereControllerEnum {BASIC, ICP, ICP_OPTIMIZATION}

   private static final SphereControllerEnum controllerType = SphereControllerEnum.ICP_OPTIMIZATION;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final GenericSphereController sphereController;

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

      switch(controllerType)
      {
      case BASIC:
         sphereController = new BasicSphereController(controlToolbox, registry);
         break;
      case ICP:
         sphereController = new SphereICPController(controlToolbox, registry);
         break;
      case ICP_OPTIMIZATION:
         sphereController = new SphereICPOptimizationController(controlToolbox, registry);
         break;
      default:
         sphereController = new BasicSphereController(controlToolbox, registry);
         break;
      }
   }

   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      robot.updateFrames();
      controlToolbox.update();

      sphereController.doControl();

      externalForcePoint.setForce(sphereController.getForces());

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
