package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class JMEDataTypeUtilsTest
{

   @Test // timeout = 30000
   public void testTransforms()
   {
      for (int i = 0; i < 1000; i++)
      {
         RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(new Random(-2346283641976L));
         //TODO @Davide test removed
         //Transform jmeVersion = JMEDataTypeUtils.j3dTransform3DToJMETransform(randomTransform);
         //RigidBodyTransform resultTransform = JMEDataTypeUtils.jmeTransformToTransform3D(jmeVersion);
         //JUnitTools.assertTransformEquals(randomTransform, resultTransform, 1e-6);
      }
   }

}
