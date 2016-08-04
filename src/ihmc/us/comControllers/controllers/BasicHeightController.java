package ihmc.us.comControllers.controllers;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;

public class BasicHeightController
{
   private static final double kp = 100.0;
   private static final double ki = 0.0;
   private static final double kd = 10.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullRobotModel robotModel;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint centerOfMass = new FramePoint();
   private final FrameVector centerOfMassVelocity = new FrameVector();

   private final YoFramePoint yoCenterOfMass = new YoFramePoint("centerOfMass", worldFrame, registry);
   private final DoubleYoVariable yoDesiredHeight = new DoubleYoVariable("desiredHeight", registry);

   private final DoubleYoVariable heightKp = new DoubleYoVariable("heightKp", registry);
   private final DoubleYoVariable heightKi = new DoubleYoVariable("heightKi", registry);
   private final DoubleYoVariable heightKd = new DoubleYoVariable("heightKd", registry);
   private final DoubleYoVariable maxIntegralError = new DoubleYoVariable("heightMaxIntegralError", registry);

   private final PIDController heightController;

   private final DoubleYoVariable verticalForce = new DoubleYoVariable("verticalForce", registry);
   private final double controlDT;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public BasicHeightController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.robotModel = controlToolbox.getFullRobotModel();
      this.controlDT = controlToolbox.getControlDT();

      centerOfMassFrame = controlToolbox.getCenterOfMassFrame();
      centerOfMassJacobian = controlToolbox.getCenterOfMassJacobian();

      yoDesiredHeight.set(controlToolbox.getDesiredHeight());

      heightKp.set(kp);
      heightKd.set(kd);
      heightKi.set(ki);
      maxIntegralError.set(Double.POSITIVE_INFINITY);

      heightController = new PIDController(heightKp, heightKi, heightKd, maxIntegralError, "heightController", registry);

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(worldFrame);

      yoCenterOfMass.set(centerOfMass);

      verticalForce.set(heightController.compute(centerOfMass.getZ(), yoDesiredHeight.getDoubleValue(), centerOfMassVelocity.getZ(), 0.0, controlDT));
   }

   public double getVerticalForce()
   {
      return verticalForce.getDoubleValue();
   }

}
