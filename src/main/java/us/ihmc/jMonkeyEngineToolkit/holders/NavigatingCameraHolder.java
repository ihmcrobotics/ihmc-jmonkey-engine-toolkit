package us.ihmc.jMonkeyEngineToolkit.holders;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;

public class NavigatingCameraHolder
{
   private CameraController navigatingCamera;

   public NavigatingCameraHolder()
   {
   }

   public void setNavigatingCamera(CameraController j3dCameraController)
   {
      navigatingCamera = j3dCameraController;
   }

   public CameraController getNavigatingCamera()
   {
      return navigatingCamera;
   }

   public boolean isThisTheNavigatingCamera(CameraController cameraToCheck)
   {
      return navigatingCamera == cameraToCheck;
   }

}
