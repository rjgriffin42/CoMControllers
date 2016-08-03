package ihmc.us.comControllers;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

import javax.vecmath.Vector3d;

public class CoMRobot extends Robot
{
   private static final long serialVersionUID = -7671864179791904256L;
   private static final double mass = 5.0;

   public CoMRobot()
   {
      super("comRobot");

      FloatingJoint floatingJoint = new FloatingJoint("floatingJoint", new Vector3d(), this);

      Link body = new Link("body");
      body.setMass(mass);
      Graphics3DObject bodyGraphics = new Graphics3DObject();
      bodyGraphics.addSphere(0.1, YoAppearance.Red());
   }

}
