package ihmc.us.comControllers.controllers;

import javax.vecmath.Vector3d;

public interface GenericSphereController
{
   public void doControl();
   public Vector3d getForces();
}
