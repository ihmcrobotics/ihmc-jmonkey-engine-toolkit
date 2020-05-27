package us.ihmc.jMonkeyEngineToolkit.camera;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class SimpleCameraTrackingAndDollyPositionHolder implements CameraTrackingAndDollyPositionHolder
{
   private final Point3D trackingPosition = new Point3D();
   private final Point3D dollyPosition = new Point3D();
   private double fieldOfView = 1.0;

   @Override
   public void getTrackingPosition(Point3DBasics trackingPositionToPack)
   {
      trackingPositionToPack.set(trackingPosition);
   }

   @Override
   public void getDollyPosition(Point3DBasics dollyPositionToPack)
   {
      dollyPositionToPack.set(dollyPosition);
   }

   @Override
   public double getTrackingX()
   {
      return trackingPosition.getX();
   }

   @Override
   public double getTrackingY()
   {
      return trackingPosition.getY();
   }

   @Override
   public double getTrackingZ()
   {
      return trackingPosition.getZ();
   }

   @Override
   public double getDollyX()
   {
      return dollyPosition.getX();
   }

   @Override
   public double getDollyY()
   {
      return dollyPosition.getY();
   }

   @Override
   public double getDollyZ()
   {
      return dollyPosition.getZ();
   }

   @Override
   public double getFieldOfView()
   {
      return fieldOfView;
   }

   public void setTrackingPosition(Point3DReadOnly trackingPosition)
   {
      this.trackingPosition.set(trackingPosition);
   }

   public void setDollyPosition(Point3DReadOnly dollyPosition)
   {
      this.dollyPosition.set(dollyPosition);
   }

   public void setTrackingX(double cameraTrackingX)
   {
      trackingPosition.setX(cameraTrackingX);
   }

   public void setTrackingY(double cameraTrackingY)
   {
      trackingPosition.setY(cameraTrackingY);
   }

   public void setTrackingZ(double cameraTrackingZ)
   {
      trackingPosition.setZ(cameraTrackingZ);
   }

   public void setDollyX(double cameraDollyX)
   {
      dollyPosition.setX(cameraDollyX);
   }

   public void setDollyY(double cameraDollyY)
   {
      dollyPosition.setY(cameraDollyY);
   }

   public void setDollyZ(double cameraDollyZ)
   {
      dollyPosition.setZ(cameraDollyZ);
   }

   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView = fieldOfView;
   }

   @Override
   public void closeAndDispose()
   {
   }

}
