package us.ihmc.jMonkeyEngineToolkit.jme;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.jMonkeyEngineToolkit.examples.Graphics3DAdapterExampleOne;

public class JMEGraphics3dAdapterTest
{

	@Test// timeout = 31000
   public void testSimpleObject()
   {
      JMEGraphics3DAdapter renderer = new JMEGraphics3DAdapter();
      Graphics3DAdapterExampleOne example1 = new Graphics3DAdapterExampleOne();

      assertTrue(example1.doExample(renderer));
   }

}
