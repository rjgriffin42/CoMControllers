package ihmc.us.comControllers;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class SphereRobot extends Robot
{
   private static final long serialVersionUID = -2136782463667249055L;

   private static final double radius = 0.5;
   private static final double mass = 1.0;
   private static final double
      Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;
   private static final double gravity = 9.81;

   private DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z;
   private DoubleYoVariable q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereEnergy");

   private final FloatingJoint floatingJoint;

   private final Vector3d initialPosition;

   public SphereRobot(String name, Vector3d initialPosition)
   {
      super(name);

      this.initialPosition = initialPosition;
      this.setGravity(0.0, 0.0, -gravity);

      // Base:

      floatingJoint = new FloatingJoint("base", new Vector3d(0.0, 0.0, 0.0), this);

      Link link1 = ball();
      floatingJoint.setLink(link1);
      this.addRootJoint(floatingJoint);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      String gcName = "gc";
      GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3d(0.0, 0.0, 0.0), this);
      floatingJoint.addGroundContactPoint(gc);

      YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(gcName + "Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereGCPoints", dynamicGraphicPosition);

      YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "Force", gc.getYoPosition(), gc.getYoForce(), 1.0/50.0);
      yoGraphicsListRegistry.registerYoGraphic("SphereForces", dynamicGraphicVector);

      initRobot();
      
      this.getRobotsYoVariableRegistry().addChild(registry);
      this.addDynamicGraphicObjectsListRegistry(yoGraphicsListRegistry);
   }

   private Link ball()
   {
      Link ret = new Link("ball");

      ret.setMass(mass);
      ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);

      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius/2.0, YoAppearance.EarthTexture());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   public void initRobot()
   {
      t.set(0.0);

      q_x = (DoubleYoVariable) this.getVariable("q_x");
      q_y = (DoubleYoVariable) this.getVariable("q_y");
      q_z = (DoubleYoVariable) this.getVariable("q_z");
      qd_x = (DoubleYoVariable) this.getVariable("qd_x");
      qd_y = (DoubleYoVariable) this.getVariable("qd_y");
      qd_z = (DoubleYoVariable) this.getVariable("qd_z");

      q_qs = (DoubleYoVariable) this.getVariable("q_qs");
      q_qx = (DoubleYoVariable) this.getVariable("q_qx");
      q_qy = (DoubleYoVariable) this.getVariable("q_qy");
      q_qz = (DoubleYoVariable) this.getVariable("q_qz");
      qd_wx = (DoubleYoVariable) this.getVariable("qd_wx");
      qd_wy = (DoubleYoVariable) this.getVariable("qd_wy");
      qd_wz = (DoubleYoVariable) this.getVariable("qd_wz");

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
