package ihmc.us.comControllers.controllers;

import ihmc.us.comControllers.model.SphereRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;

import javax.naming.Reference;

public class SphereControlToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame centerOfMassFrame;

   private final RigidBody elevator;
   private final FullRobotModel fullRobotModel;

   private final TwistCalculator twistCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final double controlDT;

   public SphereControlToolbox(FullRobotModel sphereRobotModel, double controlDT)
   {
      this.fullRobotModel = sphereRobotModel;
      this.controlDT = controlDT;

      elevator = sphereRobotModel.getElevator();

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      twistCalculator = new TwistCalculator(worldFrame, sphereRobotModel.getRootJoint().getSuccessor());
      centerOfMassJacobian = new CenterOfMassJacobian(elevator);
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public void update()
   {
      centerOfMassFrame.update();

      twistCalculator.compute();
      centerOfMassJacobian.compute();
   }
}
