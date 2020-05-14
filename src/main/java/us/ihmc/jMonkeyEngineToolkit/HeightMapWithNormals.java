package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.HeightMap;

public interface HeightMapWithNormals extends HeightMap
{
   public abstract double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack);
}
