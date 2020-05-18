package us.ihmc.jMonkeyEngineToolkit.jme;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.jMonkeyEngineToolkit.examples.Graphics3DAdapterExampleOne;

@Tag("jme")
public class JMEGraphics3dAdapterTest
{

   @Test // timeout = 31000
   public void testSimpleObject()
   {
      JMEGraphics3DAdapter renderer = new JMEGraphics3DAdapter();
      Graphics3DAdapterExampleOne example1 = new Graphics3DAdapterExampleOne();

      assertTrue(example1.doExample(renderer));
   }

}
