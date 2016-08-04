package ihmc.us.comControllers.controllers;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import javax.vecmath.Vector3d;

public class SphereICPController implements GenericSphereController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getName());

   private final SphereControlToolbox controlToolbox;
   private final BasicHeightController heightController;

   private final ICPPlanner icpPlanner;

   public SphereICPController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.controlToolbox = controlToolbox;

      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new ICPPlanner(controlToolbox.getBipedSupportPolygons(), controlToolbox.getContactableFeet(),
            controlToolbox.getCapturePointPlannerParameters(), registry, controlToolbox.getYoGraphicsListRegistry());

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
   }

   private final Vector3d forces = new Vector3d();
   public Vector3d getForces()
   {
      return forces;
   }

}
