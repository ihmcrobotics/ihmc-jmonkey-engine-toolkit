package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.Arrays;
import java.util.Random;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D32;

@Tag("jme")
public class JMEPointCloudVisualizerTest
{
   public static void main(String[] args)
   {
      new JMEPointCloudVisualizerTest().testJMEPointCloudVisualizer();
   }

   @Test // timeout = 30000
   public void testJMEPointCloudVisualizer()
   {
      JMEPointCloudVisualizer jmePointCloudVisualizer = new JMEPointCloudVisualizer();

      Random random = new Random();

      Point3D32[] randomPoint3fCloudArray = new Point3D32[10000];
      for (int i = 0; i < randomPoint3fCloudArray.length; i++)
         randomPoint3fCloudArray[i] = new Point3D32(EuclidCoreRandomTools.nextPoint3D(random, 0.0, 0.0, 0.0, 5.0, 5.0, 5.0));

      jmePointCloudVisualizer.addPointCloud(Arrays.asList(randomPoint3fCloudArray));
   }
}
