package us.ihmc.jMonkeyEngineToolkit;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddExtrusionInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;

@Tag("jme")
public class Graphics3DTextTest implements Graphics3DFrameListener
{
   int counter = 0;
   Graphics3DAddExtrusionInstruction instruction;

   @Test // timeout = 30000
   public void testTextDisplay()
   {
      Graphics3DWorld world = new Graphics3DWorld(new JMEGraphics3DAdapter());

      String text = "IHMC";

      Graphics3DObject textObject = new Graphics3DObject();
      textObject.setChangeable(true);
      textObject.rotate(-Math.PI / 2.0, Axis3D.Y);
      instruction = textObject.addText(text, 20, YoAppearance.Blue());
      Graphics3DNode textNode = new Graphics3DNode("textNode", textObject);

      world.addChild(textNode);

      world.startWithGui(1000, 800);

      world.addFrameListener(this);

      world.keepAlive(5);

      world.stop();
   }

   @Override
   public void postFrame(double timePerFrame)
   {
      if (counter % 10 == 0)
      {
         instruction.setAppearance(YoAppearance.Red());
         instruction.setText("Hello");
      }
      else if (counter % 5 == 0)
      {
         instruction.setText("IHMC!");
         instruction.setAppearance(YoAppearance.Blue());
      }

      counter++;
   }
}
