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

import static us.ihmc.robotics.Assert.assertEquals;

import java.io.IOException;
import java.time.Duration;
import java.util.List;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import com.jme3.asset.AssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.UrlAssetInfo;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DWorld;

/**
 * Test loading of STL files based on a hand crafted STL files with three triangles and known
 * values. Test both binary and ASCII versions of the same model. The model in the test files is not
 * a valid scene, and only created for easy testing of the reading code.
 *
 * @author Jesper Smith
 */
@Tag("jme")
public class STLReaderTest
{
   @Disabled
   @Test // timeout = 30000
   public void testASCIILoad() throws IOException
   {
      Assertions.assertTimeoutPreemptively(Duration.ofSeconds(20), () ->
      {
         JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
         world.startWithGui();
         ThreadTools.sleep(2);
         AssetManager assetManager = world.getGraphics3DAdapter().getRenderer().getAssetManager();
         ModelKey modelKey = new ModelKey("testASCIISTL.STL");
         UrlAssetInfo urlAssetInfo = UrlAssetInfo.create(assetManager, modelKey, getClass().getClassLoader().getResource("testASCIISTL.STL"));

         STLReader reader = STLReaderFactory.create(urlAssetInfo);
         assertEquals(reader.getClass(), ASCIISTLReader.class);

         checkData(reader);

         ThreadTools.sleep(3);
         world.stop();
      });
   }

   @Test // timeout = 30000
   public void testBinaryLoad() throws IOException
   {
      Assertions.assertTimeoutPreemptively(Duration.ofSeconds(20), () ->
      {
         JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
         world.startWithGui();
         ThreadTools.sleep(2);
         AssetManager assetManager = world.getGraphics3DAdapter().getRenderer().getAssetManager();
         ModelKey modelKey = new ModelKey("testBinarySTL.STL");
         UrlAssetInfo urlAssetInfo = UrlAssetInfo.create(assetManager, modelKey, getClass().getClassLoader().getResource("testBinarySTL.STL"));

         STLReader reader = STLReaderFactory.create(urlAssetInfo);
         assertEquals(reader.getClass(), BinarySTLReader.class);

         checkData(reader);

         ThreadTools.sleep(3);
         world.stop();
      });
   }

   private void checkData(STLReader reader)
   {
      assertEquals("TEST_CASE", reader.getName());

      List<Triangle> triangles = reader.getTriangles();
      assertEquals(3, triangles.size());
      Triangle triangle = triangles.get(0);

      assertEquals(1e3, triangle.getNormal()[0], 1e-7);
      assertEquals(0, triangle.getNormal()[1], 1e-7);
      assertEquals(0, triangle.getNormal()[2], 1e-7);
      assertEquals(1e-2, triangle.getVertex(0)[0], 1e-7);
      assertEquals(0, triangle.getVertex(0)[1], 1e-7);
      assertEquals(2e3, triangle.getVertex(0)[2], 1e-7);
      assertEquals(0, triangle.getVertex(1)[0], 1e-7);
      assertEquals(0, triangle.getVertex(1)[1], 1e-7);
      assertEquals(0, triangle.getVertex(1)[2], 1e-7);
      assertEquals(2e-5, triangle.getVertex(2)[0], 1e-7);
      assertEquals(9, triangle.getVertex(2)[1], 1e-7);
      assertEquals(2, triangle.getVertex(2)[2], 1e-7);

      triangle = triangles.get(1);
      assertEquals(0, triangle.getNormal()[0], 1e-7);
      assertEquals(1e2, triangle.getNormal()[1], 1e-7);
      assertEquals(0, triangle.getNormal()[2], 1e-7);
      assertEquals(1e-2, triangle.getVertex(0)[0], 1e-7);
      assertEquals(0, triangle.getVertex(0)[1], 1e-7);
      assertEquals(2e3, triangle.getVertex(0)[2], 1e-7);
      assertEquals(1, triangle.getVertex(1)[0], 1e-7);
      assertEquals(2, triangle.getVertex(1)[1], 1e-7);
      assertEquals(3, triangle.getVertex(1)[2], 1e-7);
      assertEquals(3, triangle.getVertex(2)[0], 1e-7);
      assertEquals(4, triangle.getVertex(2)[1], 1e-7);
      assertEquals(5, triangle.getVertex(2)[2], 1e-7);

      triangle = triangles.get(2);
      assertEquals(0, triangle.getNormal()[0], 1e-7);
      assertEquals(1e3, triangle.getNormal()[1], 1e-7);
      assertEquals(2, triangle.getNormal()[2], 1e-7);
      assertEquals(1e-2, triangle.getVertex(0)[0], 1e-7);
      assertEquals(0, triangle.getVertex(0)[1], 1e-7);
      assertEquals(2e3, triangle.getVertex(0)[2], 1e-7);
      assertEquals(4, triangle.getVertex(1)[0], 1e-7);
      assertEquals(6, triangle.getVertex(1)[1], 1e-7);
      assertEquals(7, triangle.getVertex(1)[2], 1e-7);
      assertEquals(0, triangle.getVertex(2)[0], 1e-7);
      assertEquals(2, triangle.getVertex(2)[1], 1e-7);
      assertEquals(3, triangle.getVertex(2)[2], 1e-7);
   }
}
