/**
 *
 */
package us.ihmc.jMonkeyEngineToolkit.jme.util;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import com.jme3.collision.CollisionResults;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.robotics.Assert;

/**
 * Tests for point cloud collision stuff in JMELidarSpriteGenerator
 */
@Tag("jme")
public class JMELidarSpriteGeneratorTest
{

   private JMELidarSpriteGenerator createUI(float[][] data)
   {
      JMERenderer ui = new JMERenderer(JMERenderer.RenderType.CANVAS);

      JMELidarSpriteGenerator result = new JMELidarSpriteGenerator(ui);

      ui.getZUpNode().attachChild(result);

      Point3D32[] points = new Point3D32[data.length];

      for (int i = 0; i < data.length; i++)
      {
         points[i] = new Point3D32(data[i][0], data[i][1], data[i][2]);
      }

      result.updatePoints(points);

      return result;
   }

   /**
    * Tests intersection against one point
    */

   /**
    * Not sure if we actually want this
    */
   @Disabled
   @Test // timeout = 300000
   public void testCollideWithSimple()
   {
      JMELidarSpriteGenerator generator = createUI(new float[][] {new float[] {0, 0, 0}});

      Ray ray = new Ray();
      ray.origin = new Vector3f(1, 1, 1);
      ray.direction = new Vector3f(-1, -1, -1);
      CollisionResults results = new CollisionResults();
      if (generator.collideWith(ray, results) == 0)
         Assert.fail("Collide with failed");

      assertEquals(results.size(), 1);

      assertTrue(results.getCollision(0).getContactPoint().distance(new Vector3f(0, 0, 0)) < 1e-6);

      assertEquals(results.getCollision(0).getDistance(), ray.origin.length(), 1e-6);
   }

   /**
    * Tests intersection against zero points.
    */

   @Test // timeout = 30000
   public void testEmptyCollide()
   {
      JMELidarSpriteGenerator generator = createUI(new float[][] {});
      Ray ray = new Ray();
      CollisionResults results = new CollisionResults();
      assertEquals(generator.collideWith(ray, results), 0);
   }

   /**
    * Tests multiple points lying on the same line - the closest one to the origin should be returned.
    */

   /**
    * Not sure if we actually want this
    */
   @Disabled
   @Test // timeout = 300000
   public void testMultiplePointsOneLine()
   {
      JMELidarSpriteGenerator generator = createUI(new float[][] {new float[] {0, 0, 0}, new float[] {-1, -1, -1}, new float[] {0.5f, 0.5f, 0.5f},
            new float[] {-0.5f, -0.5f, -0.5f}});
      Ray ray = new Ray();
      ray.setOrigin(new Vector3f(1, 1, 1));
      ray.setDirection(new Vector3f(-1, -1, -1));
      CollisionResults results = new CollisionResults();

      if (generator.collideWith(ray, results) == 0)
         Assert.fail("Collide with failed");

      assertEquals(1, results.size());

      assertTrue(results.getCollision(0).getContactPoint().distance(new Vector3f(0.5f, 0.5f, 0.5f)) < 1e-6);
   }

   /**
    * Tests a case when there is no intersection with the point cloud.
    */

   @Test // timeout = 30000
   public void testMultiplePointsNoMatch()
   {
      JMELidarSpriteGenerator generator = createUI(new float[][] {new float[] {10, 10, 100}, new float[] {-100, -10, -10}, new float[] {0.5f, 5.5f, 0.5f},
            new float[] {-0.5f, -0.5f, -50.5f}});

      Ray ray = new Ray();
      ray.origin = new Vector3f(1, 1, 1);
      ray.direction = new Vector3f(-1, -1, -1);
      ThreadTools.sleep(500);
      CollisionResults results = new CollisionResults();
      assertEquals(generator.collideWith(ray, results), 0);
   }

}
