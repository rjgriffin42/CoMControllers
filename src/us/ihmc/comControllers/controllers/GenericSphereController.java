package us.ihmc.comControllers.controllers;

import us.ihmc.euclid.tuple3D.Vector3D;

public interface GenericSphereController
{
   public void doControl();
   public Vector3D getForces();
}
