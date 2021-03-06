package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicReference;

import com.jme3.app.SimpleApplication;
import com.jme3.collision.Collidable;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Ray;
import com.jme3.math.Transform;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jMonkeyEngineToolkit.Updatable;

public class JMELidarSpriteGenerator extends Node implements Updatable
{
   private static final Point3D32 ORIGIN = new Point3D32();
   protected SimpleApplication jmeRenderer;
   protected Node thisObject = this;
   protected Geometry pointCloudGeometry;
   protected List<ColorRGBA> colors;
   protected ColorRGBA defaultColor;
   protected boolean newCloudAvailable = false;
   protected JMEPointCloudGenerator pointCloudGenerator;
   protected ArrayList<ColorRGBA> colorList = new ArrayList<>();
   private Random random = new Random();

   protected final AtomicReference<Point3DReadOnly[]> pointSource = new AtomicReference<>();

   public JMELidarSpriteGenerator(SimpleApplication jmeRenderer)
   {
      this(jmeRenderer, null);
   }

   public JMELidarSpriteGenerator(SimpleApplication jmeRenderer, ColorRGBA colorRGBA)
   {
      super("JMELidarFINALVisualizer");

      this.jmeRenderer = jmeRenderer;
      pointCloudGenerator = new JMEPointCloudGenerator(jmeRenderer.getAssetManager());

      defaultColor = colorRGBA;
   }

   public void clear()
   {
      jmeRenderer.enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            thisObject.detachAllChildren();

            return null;
         }
      });
   }

   public List<ColorRGBA> generateColors(Point3DReadOnly[] points)
   {
      colorList.clear();
      float distance = 3f;
      float calcDistance = 0;
      int c;

      for (Point3DReadOnly current : points)
      {
         calcDistance = (float) ORIGIN.distance(current);
         c = Color.HSBtoRGB(calcDistance % distance / distance, 1.0f, 1.0f);

         if (defaultColor == null)
         {
            colorList.add(new ColorRGBA((c >> 16 & 0xFF) / 256.0f, (c >> 8 & 0xFF) / 256.0f, (c >> 0 & 0xFF) / 256.0f, 1.0f));
         }
         else
         {
            colorList.add(defaultColor);
         }
      }

      return colorList;
   }

   public void setDefaultColor(ColorRGBA color)
   {
      defaultColor = color;
   }

   public List<ColorRGBA> generateColors(int numberOfPoints)
   {
      colorList = new ArrayList<>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         colorList.add(new ColorRGBA(random.nextFloat(), random.nextFloat(), random.nextFloat(), 1.0f));
      }

      return colorList;
   }

   public void setResolution(float resolution)
   {
      pointCloudGenerator.setSizeCM(resolution);
   }

   public void updatePoints(Point3DReadOnly[] source)
   {
      pointSource.set(source);
      newCloudAvailable = true;
      getNextCloudReady();
   }

   AtomicReference<Node> newPointCloud = new AtomicReference<>();

   public void getNextCloudReady()
   {

      if (newCloudAvailable)
      {
         //System.out.println("test3");

         newCloudAvailable = false;
         Point3DReadOnly[] pointSource = this.pointSource.get();
         if (pointSource == null)
         {
            return;
         }

         //         if (pointSource.length % 10000 <= 200)
         //            System.out.println(pointSource.length);

         colors = generateColors(pointSource);

         try
         {
            // System.out.println("making graph");
            Node pointCloud = pointCloudGenerator.generatePointCloudGraph(pointSource, colors);

            pointCloudGeometry = pointCloud.getChildren().size() > 0 ? (Geometry) pointCloud.getChild(0) : null;
            //            pointCloud.setShadowMode(ShadowMode.CastAndReceive);

            newPointCloud.set(pointCloud);
            // System.out.println("graph made");
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

   }

   public void update()
   {
      Node newCloud;

      if ((newCloud = newPointCloud.getAndSet(null)) != null)
      {
         if (getParent() != null)
         {
            thisObject.detachAllChildren();
            thisObject.attachChild(newCloud);
         }

      }
   }

   protected float getLidarResolution()
   {
      // Default to 2 cm, child classes should override
      return 0.02f;
   }

   /**
    * Whether or not to allow collisions with the LIDAR point cloud. Default set to true, child classes
    * can override.
    * 
    * @return whether point cloud collisions are allowed
    */
   protected boolean allowPointCloudCollisions()
   {
      return true;
   }

   @Override
   public int collideWith(Collidable other, CollisionResults results)
   {
      if (!allowPointCloudCollisions())
      {
         return super.collideWith(other, results);
      }

      Geometry geom;
      synchronized (pointSource)
      {
         geom = pointCloudGeometry;
      }

      // Perform a collision with the point cloud first, if no hit, move on
      if (other instanceof Ray && geom != null)
      {
         Ray ray = (Ray) other;
         if (ray.direction != null && ray.origin != null && ray.direction.lengthSquared() != 0)
         {
            Point3D32 origin = new Point3D32(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
            Vector3D32 dir = new Vector3D32(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
            Point3D32 nearest = getNearestIntersection(origin, dir, getLidarResolution(), geom.getWorldTransform());
            if (nearest != null)
            {
               CollisionResult collRes = new CollisionResult(geom,
                                                             new com.jme3.math.Vector3f(nearest.getX32(), nearest.getY32(), nearest.getZ32()),
                                                             (float) origin.distance(nearest),
                                                             0);
               collRes.setContactNormal(new com.jme3.math.Vector3f(0, 0, 1));
               results.addCollision(collRes);

               return 1;
            }
            else
            {
               return 0;
            }
         }
      }

      return super.collideWith(other, results);
   }

   /**
    * Find the nearest point along the ray that is also closest to the ray origin.
    * 
    * @param origin     ray origin
    * @param direction  ray direction
    * @param resolution lidar resolution
    * @return closest point or null if the ray did not hit any lidar point
    */
   public Point3D32 getNearestIntersection(Point3DReadOnly origin, Vector3DReadOnly direction, double resolution, Transform pointTransform)
   {
      Point3DReadOnly[] points = pointSource.get();
      direction = new UnitVector3D(direction);
      float dx, dy, dz, dot;
      double distanceToLine, distance;

      double nearestDistance = Double.POSITIVE_INFINITY;
      Point3D32 nearestPoint = null;

      for (Point3DReadOnly p1 : points)
      {
         com.jme3.math.Vector3f p = new com.jme3.math.Vector3f(p1.getX32(), p1.getY32(), p1.getZ32());
         pointTransform.transformVector(p, p);

         dx = origin.getX32() - p.x;
         dy = origin.getY32() - p.y;
         dz = origin.getZ32() - p.z;

         dot = dx * direction.getX32() + dy * direction.getY32() + dz * direction.getZ32();

         dx = dx - dot * direction.getX32();
         dy = dy - dot * direction.getY32();
         dz = dz - dot * direction.getZ32();

         distanceToLine = dx * dx + dy * dy + dz * dz;

         Point3D32 curpt = new Point3D32(p.x, p.y, p.z);

         if (distanceToLine < resolution * resolution)
         {
            distance = origin.distanceSquared(curpt);

            if (distance < nearestDistance)
            {
               nearestDistance = distance;
               nearestPoint = curpt;
            }
         }
      }

      return nearestPoint;
   }

   @Override
   public void simpleUpdate(float tpf)
   {
      update();
   }
}
