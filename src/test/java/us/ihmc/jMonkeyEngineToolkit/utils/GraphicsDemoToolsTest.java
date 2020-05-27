package us.ihmc.jMonkeyEngineToolkit.utils;

import static us.ihmc.robotics.Assert.assertNotNull;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

public class GraphicsDemoToolsTest
{

   @Test // timeout = 30000
   public void testCreatePointCloud()
   {
      List<Point3D> worldPoints = new ArrayList<>();

      for (int i = 0; i < 1000; i++)
      {
         worldPoints.add(new Point3D(1.0, 1.0, 1.0));
      }

      Graphics3DNode pointCloudNode = GraphicsDemoTools.createPointCloud("PointCloud", worldPoints, 0.001, YoAppearance.Green());

      assertNotNull("Point cloud node is null. ", pointCloudNode);
   }
}
