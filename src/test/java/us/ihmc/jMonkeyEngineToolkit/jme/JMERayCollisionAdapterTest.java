package us.ihmc.jMonkeyEngineToolkit.jme;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

@Tag("jme")
public class JMERayCollisionAdapterTest
{
   @Disabled
   @Test // timeout=300000
   public void testObjectPicking()
   {
      //      ThreadTools.sleep(10000); // Put this in to give me time to attach the debugger to this test.
      JMEGraphics3DWorld world = new JMEGraphics3DWorld(new JMEGraphics3DAdapter());
      world.startWithGui();

      double CUBE_SIDE = 2.0;
      double CUBE_X = 2.0;
      double ERROR_TOLERANCE = 1e-6;
      double EXPECTED_BOX_CONTACT_X = 1.0;

      Graphics3DObject cubeGraphics = new Graphics3DObject();
      cubeGraphics.translate(new Vector3D(CUBE_X, 0, -1));
      cubeGraphics.addCube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

      world.addChild(new Graphics3DNode("CubeNode", cubeGraphics));

      world.keepAlive(0.1); // Needs to happen so scene graph initializes properly!

      Line3D ray3d = new Line3D(new Point3D(0.0, 0.0, 0.0), new Vector3D(1.0, 0.0, 0.0));

      JMERayCollisionAdapter rayCollisionAdapter = new JMERayCollisionAdapter(world.getJMERootNode());
      rayCollisionAdapter.setPickingGeometry(ray3d);
      double pickDistance = rayCollisionAdapter.getPickDistance();

      assertEquals(EXPECTED_BOX_CONTACT_X, pickDistance, ERROR_TOLERANCE);

      world.stop();
   }
}
