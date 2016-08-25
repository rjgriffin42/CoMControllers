package us.ihmc.comControllers.model;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class SphereRobot
{
   private static final double radius = 0.1;

   public static RobotTools.SCSRobotFromInverseDynamicsRobotModel createSphereRobot(String name, Vector3d initialPosition, RigidBody elevator,
         YoGraphicsListRegistry yoGraphicsListRegistry, double gravity)
   {
      RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot = new RobotTools.SCSRobotFromInverseDynamicsRobotModel(name, elevator.getChildrenJoints().get(0));

      scsRobot.setGravity(0.0, 0.0, -gravity);

      Joint floatingJoint = scsRobot.getRootJoints().get(0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius/2.0, YoAppearance.EarthTexture());
      floatingJoint.getLink().setLinkGraphics(linkGraphics);

      String gcName = "gc";
      GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3d(0.0, 0.0, 0.0), scsRobot);
      floatingJoint.addGroundContactPoint(gc);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("forcePoint", new Vector3d(), scsRobot);
      floatingJoint.addExternalForcePoint(externalForcePoint);

      /*
      YoGraphicVector externalForceGraphic = new YoGraphicVector(externalForcePoint.getName() + "Force", externalForcePoint.getYoPosition(),
            externalForcePoint.getYoForce(), 1.0/10.0);
      yoGraphicsListRegistry.registerYoGraphic("ExternalForces", externalForceGraphic);
      */

      YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(gcName + "Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereGCPoints", dynamicGraphicPosition);

      YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "Force", gc.getYoPosition(), gc.getYoForce(), 1.0/50.0);
      yoGraphicsListRegistry.registerYoGraphic("SphereForces", dynamicGraphicVector);

      initRobot(scsRobot, initialPosition);
      
      scsRobot.addDynamicGraphicObjectsListRegistry(yoGraphicsListRegistry);
      scsRobot.update();

      return scsRobot;
   }

   public static void initRobot(Robot scsRobot, Vector3d initialPosition)
   {
      scsRobot.getYoTime().set(0.0);

      DoubleYoVariable q_x = (DoubleYoVariable) scsRobot.getVariable("q_x");
      DoubleYoVariable q_y = (DoubleYoVariable) scsRobot.getVariable("q_y");
      DoubleYoVariable q_z = (DoubleYoVariable) scsRobot.getVariable("q_z");
      DoubleYoVariable qd_x = (DoubleYoVariable) scsRobot.getVariable("qd_x");
      DoubleYoVariable qd_y = (DoubleYoVariable) scsRobot.getVariable("qd_y");
      DoubleYoVariable qd_z = (DoubleYoVariable) scsRobot.getVariable("qd_z");

      DoubleYoVariable q_qs = (DoubleYoVariable) scsRobot.getVariable("q_qs");
      DoubleYoVariable q_qx = (DoubleYoVariable) scsRobot.getVariable("q_qx");
      DoubleYoVariable q_qy = (DoubleYoVariable) scsRobot.getVariable("q_qy");
      DoubleYoVariable q_qz = (DoubleYoVariable) scsRobot.getVariable("q_qz");
      DoubleYoVariable qd_wx = (DoubleYoVariable) scsRobot.getVariable("qd_wx");
      DoubleYoVariable qd_wy = (DoubleYoVariable) scsRobot.getVariable("qd_wy");
      DoubleYoVariable qd_wz = (DoubleYoVariable) scsRobot.getVariable("qd_wz");

      q_x.set(initialPosition.getX());
      q_y.set(initialPosition.getY());
      q_z.set(initialPosition.getZ());

      qd_x.set(0.0);
      qd_y.set(0.0);
      qd_z.set(-0.10);

      q_qs.set(0.707106);
      q_qx.set(0.0);
      q_qy.set(0.707106);
      q_qz.set(0.0);

      qd_wx.set(0.0);
      qd_wy.set(0.0);
      qd_wz.set(0.0);
   }
}
