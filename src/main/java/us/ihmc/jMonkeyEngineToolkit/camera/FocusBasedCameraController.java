package us.ihmc.jMonkeyEngineToolkit.camera;

import com.jme3.math.Vector3f;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.mouse.MouseButton;
import us.ihmc.graphicsDescription.input.mouse.MouseListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jme.JMEPoseReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.tools.inputDevices.keyboard.Key;

import javax.swing.*;
import java.util.ArrayList;

public class FocusBasedCameraController implements TrackingDollyCameraController
{
   private final JMEPoseReferenceFrame zUpFrame = new JMEPoseReferenceFrame("ZUpFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D cameraPose = new FramePose3D();
   private final RigidBodyTransform cameraTransform = new RigidBodyTransform();

   private final AxisAngle latitudeAxisAngle = new AxisAngle();
   private final AxisAngle longitudeAxisAngle = new AxisAngle();

   private final RotationMatrix cameraOrientationOffset = new RotationMatrix();

   private double zoomSpeedFactor = 0.1;
   private double latitudeSpeed = 5.0;
   private double longitudeSpeed = 5.0;
   private double translateSpeed = 5.0;

   private final FramePose3D focusPointPose = new FramePose3D();
   private final Graphics3DNode fixPointNode = new Graphics3DNode("cameraFixPoint",
                                                                  new Graphics3DObject(new Sphere3D(0.01),
                                                                                       YoAppearance.RGBColor(1.0, 0.0, 0.0, 0.5)));
   private double latitude = 0.0;
   private double longitude = 0.0;
   private double roll;
   private double zoom = 10.0;

   private final Vector3D up;
   private final Vector3D forward;
   private final Vector3D left;
   private final Vector3D down;

   private final Vector3f translationJME = new Vector3f();
   private final com.jme3.math.Quaternion orientationJME = new com.jme3.math.Quaternion();

   private boolean leftMousePressed = false;
   private boolean isWPressed = false;
   private boolean isAPressed = false;
   private boolean isSPressed = false;
   private boolean isDPressed = false;
   private boolean isQPressed = false;
   private boolean isZPressed = false;

   private final float fov = 45.0f;
   private final float nearClip = 0.05f;
   private final float farClip = 2000.0f;

   private final CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder;

   private KeyListener keyListener = new PrivateKeyListener();
   private MouseListener mouseListener = new PrivateMouseListener();
   private final RotationMatrix zUpToYUp;

   private RigidBodyTransform focusPointTransform = new RigidBodyTransform();

   public FocusBasedCameraController(Graphics3DAdapter graphics3dAdapter,
                                     ViewportAdapter viewportAdapter,
                                     CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder,
                                     JFrame jFrame,
                                     boolean addListeners)
   {
      this.cameraTrackAndDollyVariablesHolder = cameraTrackAndDollyVariablesHolder;
      //      setFrustumPerspective(fov, (float) width / height, nearClip, farClip);

      JMEGraphics3DAdapter jmeGraphics3DAdapter = (JMEGraphics3DAdapter) graphics3dAdapter;

      zUpToYUp = new RotationMatrix();
      zUpToYUp.set(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0);
      zUpFrame.setOrientationAndUpdate(zUpToYUp);

      up = new Vector3D(0.0, 0.0, 1.0);
      forward = new Vector3D(1.0, 0.0, 0.0);
      left = new Vector3D();
      left.cross(up, forward);
      down = new Vector3D();
      down.setAndNegate(up);
      Vector3D cameraZAxis = new Vector3D(forward);
      Vector3D cameraYAxis = new Vector3D(up);
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(cameraYAxis, cameraZAxis);
      cameraOrientationOffset.setColumns(cameraXAxis, cameraYAxis, cameraZAxis);

      changeCameraPosition(-2.0, 0.7, 1.0);

      updateCameraPose();

      jmeGraphics3DAdapter.addRootNode(fixPointNode);

      if (addListeners)
      {
         graphics3dAdapter.addKeyListener(keyListener);
         graphics3dAdapter.addMouseListener(mouseListener);
      }
   }

   public void changeCameraPosition(double x, double y, double z)
   {
      Point3D desiredCameraPosition = new Point3D(x, y, z);

      zoom = desiredCameraPosition.distance(focusPointPose.getPosition());

      Vector3D fromFocusToCamera = new Vector3D();
      fromFocusToCamera.sub(desiredCameraPosition, focusPointPose.getPosition());
      fromFocusToCamera.normalize();
      Vector3D fromCameraToFocus = new Vector3D();
      fromCameraToFocus.setAndNegate(fromFocusToCamera);
      // We remove the component along up to be able to compute the longitude
      fromCameraToFocus.scaleAdd(-fromCameraToFocus.dot(down), down, fromCameraToFocus);

      latitude = Math.PI / 2.0 - fromFocusToCamera.angle(down);
      longitude = fromCameraToFocus.angle(forward);

      Vector3D cross = new Vector3D();
      cross.cross(fromCameraToFocus, forward);

      if (cross.dot(down) > 0.0)
         longitude = -longitude;
   }

   @Override
   public void computeTransform(RigidBodyTransform cameraTransform, float tpf)
   {
      if (isWPressed)
      {
         focusPointPose.appendTranslation(translateSpeed * tpf, 0.0, 0.0);
      }
      if (isAPressed)
      {
         focusPointPose.appendTranslation(0.0, translateSpeed * tpf, 0.0);
      }
      if (isSPressed)
      {
         focusPointPose.appendTranslation(-translateSpeed * tpf, 0.0, 0.0);
      }
      if (isDPressed)
      {
         focusPointPose.appendTranslation(0.0, -translateSpeed * tpf, 0.0);
      }
      if (isQPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, translateSpeed * tpf);
      }
      if (isZPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, -translateSpeed * tpf);
      }

      updateCameraPose();

      focusPointPose.get(focusPointTransform);
      fixPointNode.setTransform(focusPointTransform);

      // cam appears to be x forward
      cameraTransform.set(this.cameraTransform);
   }

   private void updateCameraPose()
   {
      zoom = MathTools.clamp(zoom, 0.1, 100.0);
      latitude = MathTools.clamp(latitude, Math.PI / 1.99); // don't let it get close to the singularities
      longitude = EuclidCoreTools.trimAngleMinusPiToPi(longitude);

      longitudeAxisAngle.set(Axis3D.Z, -longitude);
      focusPointPose.getOrientation().set(longitudeAxisAngle);

      cameraTransform.setToZero();
      cameraTransform.appendTranslation(focusPointPose.getPosition());
      cameraTransform.appendYawRotation(-longitude);
      cameraTransform.appendPitchRotation(-latitude);
      cameraTransform.appendTranslation(-zoom, 0.0, 0.0);
   }

   class PrivateMouseListener implements MouseListener
   {
      @Override
      public void mouseDragged(MouseButton mouseButton, double dx, double dy)
      {
         // undo mouseFactor from JMEInputManager
         dx /= 10.0;
         dy /= -10.0;

         switch (mouseButton)
         {
            case LEFT:
               latitude += latitudeSpeed * dy;
               longitude += longitudeSpeed * dx;
               break;
            case RIGHT:
               break;
            case MIDDLE:
               break;
            case LEFTRIGHT:
               break;
         }
      }

      @Override
      public void scrolled(double amount)
      {
         if (amount > 0.0)
         {
            LogTools.info("Zoom in");
            zoom = zoom - zoom * zoomSpeedFactor;
         }
         else
         {
            LogTools.info("Zoom out");
            zoom = zoom + zoom * zoomSpeedFactor;
         }
      }
   }

   class PrivateKeyListener implements KeyListener
   {
      @Override
      public void keyPressed(Key key)
      {
         switch (key)
         {
            case W:
               isWPressed = true;
               break;
            case S:
               isSPressed = true;
               break;
            case A:
               isAPressed = true;
               break;
            case D:
               isDPressed = true;
               break;
            case Q:
               isQPressed = true;
               break;
            case Z:
               isZPressed = true;
               break;
            default:
               break;
         }
      }

      @Override
      public void keyReleased(Key key)
      {
         switch (key)
         {
            case W:
               isWPressed = false;
               break;
            case S:
               isSPressed = false;
               break;
            case A:
               isAPressed = false;
               break;
            case D:
               isDPressed = false;
               break;
            case Q:
               isQPressed = false;
               break;
            case Z:
               isZPressed = false;
               break;
            default:
               break;
         }
      }
   }

   @Override
   public double getHorizontalFieldOfViewInRadians()
   {
      return fov;
   }

   @Override
   public double getClipNear()
   {
      return nearClip;
   }

   @Override
   public double getClipFar()
   {
      return farClip;
   }

   @Override
   public void closeAndDispose()
   {

   }

   @Override
   public boolean isTracking()
   {
      return false;
   }

   @Override
   public boolean isTrackingX()
   {
      return false;
   }

   @Override
   public boolean isTrackingY()
   {
      return false;
   }

   @Override
   public boolean isTrackingZ()
   {
      return false;
   }

   @Override
   public boolean isDolly()
   {
      return false;
   }

   @Override
   public boolean isDollyX()
   {
      return false;
   }

   @Override
   public boolean isDollyY()
   {
      return false;
   }

   @Override
   public boolean isDollyZ()
   {
      return false;
   }

   @Override
   public void setTracking(boolean track)
   {

   }

   @Override
   public void setTrackingX(boolean trackX)
   {

   }

   @Override
   public void setTrackingY(boolean trackY)
   {

   }

   @Override
   public void setTrackingZ(boolean trackZ)
   {

   }

   @Override
   public void setDolly(boolean dolly)
   {

   }

   @Override
   public void setDollyX(boolean dollyX)
   {

   }

   @Override
   public void setDollyY(boolean dollyY)
   {

   }

   @Override
   public void setDollyZ(boolean dollyZ)
   {

   }

   @Override
   public double getTrackingXOffset()
   {
      return 0;
   }

   @Override
   public double getTrackingYOffset()
   {
      return 0;
   }

   @Override
   public double getTrackingZOffset()
   {
      return 0;
   }

   @Override
   public double getDollyXOffset()
   {
      return 0;
   }

   @Override
   public double getDollyYOffset()
   {
      return 0;
   }

   @Override
   public double getDollyZOffset()
   {
      return 0;
   }

   @Override
   public void setTrackingXOffset(double dx)
   {

   }

   @Override
   public void setTrackingYOffset(double dy)
   {

   }

   @Override
   public void setTrackingZOffset(double dz)
   {

   }

   @Override
   public void setDollyXOffset(double dx)
   {

   }

   @Override
   public void setDollyYOffset(double dy)
   {

   }

   @Override
   public void setDollyZOffset(double dz)
   {

   }

   @Override
   public void setFieldOfView(double fieldOfView)
   {

   }

   @Override
   public void setClipDistanceNear(double near)
   {

   }

   @Override
   public void setClipDistanceFar(double far)
   {

   }

   @Override
   public double getFixX()
   {
      return focusPointPose.getX();
   }

   @Override
   public double getFixY()
   {
      return focusPointPose.getY();
   }

   @Override
   public double getFixZ()
   {
      return focusPointPose.getZ();
   }

   @Override
   public double getCamX()
   {
      return cameraPose.getX();
   }

   @Override
   public double getCamY()
   {
      return cameraPose.getY();
   }

   @Override
   public double getCamZ()
   {
      return cameraPose.getZ();
   }

   @Override
   public void setFixX(double fx)
   {

   }

   @Override
   public void setFixY(double fy)
   {

   }

   @Override
   public void setFixZ(double fz)
   {

   }

   @Override
   public void setCamX(double cx)
   {

   }

   @Override
   public void setCamY(double cy)
   {

   }

   @Override
   public void setCamZ(double cz)
   {

   }

   @Override
   public double getTrackXVar()
   {
      return 0;
   }

   @Override
   public double getTrackYVar()
   {
      return 0;
   }

   @Override
   public double getTrackZVar()
   {
      return 0;
   }

   @Override
   public double getDollyXVar()
   {
      return 0;
   }

   @Override
   public double getDollyYVar()
   {
      return 0;
   }

   @Override
   public double getDollyZVar()
   {
      return 0;
   }

   @Override
   public void update()
   {

   }

   @Override
   public void reset()
   {

   }

   @Override
   public void setTrackingOffsets(double dx, double dy, double dz)
   {

   }

   @Override
   public void setDollyOffsets(double dx, double dy, double dz)
   {

   }

   @Override
   public void setFixPosition(double fixX, double fixY, double fixZ)
   {

   }

   @Override
   public void setCameraPosition(double posX, double posY, double posZ)
   {
//      changeCameraPosition(posX, posY, posZ);
   }

   @Override
   public void setTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
   {

   }

   @Override
   public void setDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
   {

   }

   @Override
   public CameraTrackingAndDollyPositionHolder getCameraTrackAndDollyVariablesHolder()
   {
      return cameraTrackAndDollyVariablesHolder;
   }

   @Override
   public void setConfiguration(CameraConfiguration config, CameraMountList mountList)
   {

   }

   @Override
   public boolean setCameraKeyPoint(int index)
   {
      return false;
   }

   @Override
   public void removeCameraKeyPoint(int index)
   {

   }

   @Override
   public void nextCameraKeyPoint(int index)
   {

   }

   @Override
   public void previousCameraKeyPoint(int index)
   {

   }

   @Override
   public void toggleCameraKeyMode()
   {

   }

   @Override
   public boolean useKeyCameraPoints()
   {
      return false;
   }

   @Override
   public void setKeyFrameTime(int index)
   {

   }

   @Override
   public void setUseCameraKeyPoints(boolean b)
   {

   }

   @Override
   public void copyPositionTrackingDollyConfiguration(TrackingDollyCameraController otherCamera)
   {

   }

   @Override
   public Graphics3DNode getFixPointNode()
   {
      return null;
   }

   @Override
   public ArrayList<Integer> getCameraKeyPoints()
   {
      return null;
   }

   @Override
   public boolean getCameraKeyMode()
   {
      return false;
   }
}
