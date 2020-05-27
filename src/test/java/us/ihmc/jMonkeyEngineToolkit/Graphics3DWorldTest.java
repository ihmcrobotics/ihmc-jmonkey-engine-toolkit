package us.ihmc.jMonkeyEngineToolkit;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;

@Tag("jme")
public class Graphics3DWorldTest
{

   @Test // timeout = 30000
   public void testShowGui()
   {
      Graphics3DWorld world = new Graphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      world.startWithGui();
      world.keepAlive(1.0);
      world.stop();
   }

   @Test // timeout = 30000
   public void testWithoutGui()
   {
      Graphics3DWorld world = new Graphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      world.startWithoutGui();
      world.keepAlive(1.0);
      world.stop();
   }

   @Test // timeout = 30000
   public void addASphere()
   {
      Graphics3DWorld world = new Graphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      world.startWithoutGui();
      world.addChild(new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3D(), YoAppearance.Glass())));
      world.keepAlive(1.0);
      world.stop();
   }

   @Test // timeout = 30000
   public void addASphereAfterGuiStarted()
   {
      Graphics3DWorld world = new Graphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      world.startWithoutGui();
      world.addChild(new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3D())));
      world.keepAlive(1.0);
      world.stop();
   }

   @Test // timeout = 30000
   public void testSetCameraPosition()
   {
      Graphics3DWorld world = new Graphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      world.startWithGui();
      world.addChild(new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3D(), YoAppearance.Glass(0.2))));
      world.setCameraPosition(5, 5, 5);
      world.keepAlive(1.0);
      world.stop();
   }

   @Test // timeout = 30000
   public void fixCameraOnSphere()
   {
      Graphics3DWorld world = new Graphics3DWorld("testWorld", new JMEGraphics3DAdapter());

      Graphics3DNode sphereNode = new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3D()));
      world.startWithGui();
      world.addChild(sphereNode);
      world.setCameraPosition(5, 5, 5);
      world.fixCameraOnNode(sphereNode);

      world.keepAlive(1.0);
      world.stop();
   }
}
