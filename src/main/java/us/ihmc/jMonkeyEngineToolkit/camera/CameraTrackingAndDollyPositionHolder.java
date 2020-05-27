package us.ihmc.jMonkeyEngineToolkit.camera;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface CameraTrackingAndDollyPositionHolder
{
   public abstract void getTrackingPosition(Point3DBasics trackPositionToPack);

   public abstract void getDollyPosition(Point3DBasics dollyPositionToPack);

   public abstract double getTrackingX();

   public abstract double getTrackingY();

   public abstract double getTrackingZ();

   public abstract double getDollyX();

   public abstract double getDollyY();

   public abstract double getDollyZ();

   public abstract double getFieldOfView();

   public abstract void closeAndDispose();

}
