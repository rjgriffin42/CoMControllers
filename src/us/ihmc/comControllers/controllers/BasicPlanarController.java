package us.ihmc.comControllers.controllers;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;

import javax.vecmath.Point2d;

public class BasicPlanarController
{
   private static final double desiredX = 0.0;
   private static final double desiredY = 0.0;

   private static final double kp = 10.0;
   private static final double ki = 0.0;
   private static final double kd = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullRobotModel robotModel;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint centerOfMass = new FramePoint();
   private final FrameVector centerOfMassVelocity = new FrameVector();

   private final DoubleYoVariable yoDesiredX = new DoubleYoVariable("desiredX", registry);
   private final DoubleYoVariable yoDesiredY = new DoubleYoVariable("desiredY", registry);

   private final DoubleYoVariable xKp = new DoubleYoVariable("planarXKp", registry);
   private final DoubleYoVariable xKi = new DoubleYoVariable("planarXKi", registry);
   private final DoubleYoVariable xKd = new DoubleYoVariable("planarXKd", registry);
   private final DoubleYoVariable xMaxIntegralError = new DoubleYoVariable("xMaxIntegralError", registry);
   private final DoubleYoVariable yKp = new DoubleYoVariable("planarYKp", registry);
   private final DoubleYoVariable yKi = new DoubleYoVariable("planarYKi", registry);
   private final DoubleYoVariable yKd = new DoubleYoVariable("planarYKd", registry);
   private final DoubleYoVariable yMaxIntegralError = new DoubleYoVariable("yMaxIntegralError", registry);

   private final PIDController xController;
   private final PIDController yController;

   private final YoFramePoint2d planarForces = new YoFramePoint2d("planarForces", null, registry);
   private final double controlDT;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public BasicPlanarController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.robotModel = controlToolbox.getFullRobotModel();
      this.controlDT = controlToolbox.getControlDT();

      centerOfMassFrame = controlToolbox.getCenterOfMassFrame();
      centerOfMassJacobian = controlToolbox.getCenterOfMassJacobian();

      yoDesiredX.set(desiredX);
      yoDesiredY.set(desiredY);

      xKp.set(kp);
      xKd.set(kd);
      xKi.set(ki);
      xMaxIntegralError.set(Double.POSITIVE_INFINITY);
      yKp.set(kp);
      yKd.set(kd);
      yKi.set(ki);
      yMaxIntegralError.set(Double.POSITIVE_INFINITY);

      xController = new PIDController(xKp, xKi, xKd, xMaxIntegralError, "xController", registry);
      yController = new PIDController(yKp, yKi, yKd, yMaxIntegralError, "yController", registry);

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(worldFrame);

      double xForce = xController.compute(centerOfMass.getX(), yoDesiredX.getDoubleValue(), centerOfMassVelocity.getX(), 0.0, controlDT);
      double yForce = yController.compute(centerOfMass.getY(), yoDesiredY.getDoubleValue(), centerOfMassVelocity.getY(), 0.0, controlDT);
      planarForces.set(xForce, yForce);
   }

   public void getPlanarForces(Point2d forces)
   {
      forces.set(planarForces.getPoint2dCopy());
   }

}
