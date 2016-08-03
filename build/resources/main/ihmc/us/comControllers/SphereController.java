package ihmc.us.comControllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class SphereController implements RobotController
{
   private static final long serialVersionUID = -6115066729570319285L;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");


   private final SphereRobot robot;
   
   public SphereController(SphereRobot robot)
   {
      this.robot = robot;
   }
   

   public void doControl()
   {
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
