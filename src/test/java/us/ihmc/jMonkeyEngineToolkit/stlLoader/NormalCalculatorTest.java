/*
 *   Copyright 2014 Florida Institute for Human and Machine Cognition (IHMC)
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *    Written by Jesper Smith with assistance from IHMC team members
 */
package us.ihmc.jMonkeyEngineToolkit.stlLoader;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.IOException;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import com.jme3.asset.AssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.UrlAssetInfo;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DWorld;

/**
 * Test the normal calculation routine based on the normals in the the teapotBinary.STL model.
 *
 * @author Jesper Smith
 */
@Tag("jme")
public class NormalCalculatorTest
{

   @Test // timeout = 30000
   public void testNormalsBasedOnTeapot() throws IOException
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      world.startWithGui();
      ThreadTools.sleep(2);
      AssetManager assetManager = world.getGraphics3DAdapter().getRenderer().getAssetManager();
      ModelKey modelKey = new ModelKey("teapotBinary.STL");
      UrlAssetInfo urlAssetInfo = UrlAssetInfo.create(assetManager, modelKey, getClass().getClassLoader().getResource("teapotBinary.STL"));

      STLReader reader = STLReaderFactory.create(urlAssetInfo);

      for (Triangle triangle : reader.getTriangles())
      {
         float[] normal = triangle.getNormal();
         float[] calculatedNormal = new float[3];
         NormalCalculator.calculateNormal(calculatedNormal, triangle.getVertices());

         assertTrue(NormalCalculator.compareNormal(normal, calculatedNormal, 1e-2f));
      }

      ThreadTools.sleep(3);
      world.stop();
   }
}
