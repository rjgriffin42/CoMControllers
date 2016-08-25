package us.ihmc.comControllers.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

public class BasicSphereController implements GenericSphereController
{
   private final BasicHeightController heightController;
   private final BasicPlanarController planarController;

   public BasicSphereController(SphereControlToolbox controlToolbox, YoVariableRegistry registry)
   {
      heightController = new BasicHeightController(controlToolbox, registry);
      planarController = new BasicPlanarController(controlToolbox, registry);
   }

   private final Point2d planarForces = new Point2d();
   private final Vector3d forces = new Vector3d();
   public void doControl()
   {
      heightController.doControl();
      planarController.doControl();
      planarController.getPlanarForces(planarForces);

      forces.set(planarForces.getX(), planarForces.getY(), heightController.getVerticalForce());
   }

   public Vector3d getForces()
   {
      return forces;
   }
}
