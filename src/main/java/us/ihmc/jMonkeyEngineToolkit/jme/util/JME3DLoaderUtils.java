package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.ArrayList;
import java.util.List;

import com.jme3.asset.AssetManager;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;

import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphicsObject;

public class JME3DLoaderUtils
{
   public static Spatial load3DModel(String modelPath, AssetManager assetManager, Graphics3DNodeType type)
   {
      return JMEGraphicsObject.createGraphics3DObjectFromModel(modelPath, null, false, null, assetManager);
   }

   public static ArrayList<Geometry> extractGeometry(Spatial spatial)
   {
      ArrayList<Geometry> geom = new ArrayList<>();
      getGeometry(spatial, geom);

      return geom;
   }

   public static Geometry extractFirstGeometry(String modelPath, AssetManager assetManager, Graphics3DNodeType type)
   {
      Spatial tmp = load3DModel(modelPath, assetManager, type);

      return extractFirstGeometry(tmp);
   }

   public static Geometry extractFirstGeometry(Spatial spatial)
   {
      List<Geometry> geom = new ArrayList<>();
      getGeometry(spatial, geom);

      return geom.get(0);
   }

   private static void getGeometry(Spatial spatial, List<Geometry> parent)
   {
      if (spatial instanceof Node)
      {
         Node n = (Node) spatial;
         for (Spatial child : n.getChildren())
         {
            getGeometry(child, parent);
         }
      }
      else if (spatial instanceof Geometry)
      {
         parent.add((Geometry) spatial);
      }
   }
}
