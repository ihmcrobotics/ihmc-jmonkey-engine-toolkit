package us.ihmc.jMonkeyEngineToolkit.camera;

import java.util.ArrayList;
import java.util.List;

public class CameraMountList implements java.io.Serializable
{
   private static final long serialVersionUID = 7819849315544602348L;
   private final List<CameraMountInterface> mounts = new ArrayList<>();

   public CameraMountList()
   {
   }

   public void addCameraMount(CameraMountInterface mount)
   {
      mounts.add(mount);
   }

   public void addCameraMounts(List<CameraMountInterface> mountArrayList)
   {
      mounts.addAll(mountArrayList);
   }

   public CameraMountInterface getCameraMount(String name)
   {
      for (int i = 0; i < mounts.size(); i++)
      {
         CameraMountInterface mount = mounts.get(i);

         if (mount.getName().equals(name))
         {
            return mount;
         }
      }

      return null;
   }

   public List<CameraMountInterface> getCameraMountList()
   {
      return mounts;
   }
}
