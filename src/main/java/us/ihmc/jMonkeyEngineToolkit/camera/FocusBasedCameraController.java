package us.ihmc.jMonkeyEngineToolkit.camera;

import com.jme3.asset.AssetManager;
import com.jme3.input.InputManager;
import com.jme3.math.Vector3f;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.mouse.MouseButton;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jme.JMEPoseReferenceFrame;
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
   private final AxisAngle rollAxisAngle = new AxisAngle();

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

//   private final Geometry focusPointSphere;

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

   private KeyListener keyListener;

   public FocusBasedCameraController(Graphics3DAdapter graphics3dAdapter,
                                     ViewportAdapter viewportAdapter,
                                     CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder,
                                     JFrame jFrame,
                                     boolean addListeners)
   {
      this.cameraTrackAndDollyVariablesHolder = cameraTrackAndDollyVariablesHolder;
      //      setFrustumPerspective(fov, (float) width / height, nearClip, farClip);

      JMEGraphics3DAdapter jmeGraphics3DAdapter = (JMEGraphics3DAdapter) graphics3dAdapter;
      AssetManager assetManager = jmeGraphics3DAdapter.getRenderer().getAssetManager();
      InputManager inputManager = jmeGraphics3DAdapter.getRenderer().getInputManager();

      RotationMatrix zUpToYUp = new RotationMatrix();
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

//      JMEMultiColorMeshBuilder colorMeshBuilder = new JMEMultiColorMeshBuilder();
//      colorMeshBuilder.addSphere((float) 1.0, new Point3D(0.0, 0.0, 0.0), Color.DARKRED);
//      focusPointSphere = new Geometry("FocusPointViz", colorMeshBuilder.generateMesh());
//      focusPointSphere.setMaterial(colorMeshBuilder.generateMaterial(assetManager));

      focusPointPose.changeFrame(zUpFrame);
      changeCameraPosition(-2.0, 0.7, 1.0);

      updateCameraPose();

//      JMEInputMapperHelper inputMapper = new JMEInputMapperHelper(inputManager);
//      inputMapper.addAnalogMapping("onMouseYUp", new MouseAxisTrigger(MouseInput.AXIS_Y, false), this::onMouseYUp);
//      inputMapper.addAnalogMapping("onMouseYDown", new MouseAxisTrigger(MouseInput.AXIS_Y, true), this::onMouseYDown);
//      inputMapper.addAnalogMapping("onMouseXLeft", new MouseAxisTrigger(MouseInput.AXIS_X, true), this::onMouseXLeft);
//      inputMapper.addAnalogMapping("onMouseXRight", new MouseAxisTrigger(MouseInput.AXIS_X, false), this::onMouseXRight);
//      inputMapper.addAnalogMapping("onMouseScrollUp", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false), this::onMouseScrollUp);
//      inputMapper.addAnalogMapping("onMouseScrollDown", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true), this::onMouseScrollDown);
//      inputMapper.addActionMapping("onMouseButtonLeft", new MouseButtonTrigger(MouseInput.BUTTON_LEFT), this::onMouseButtonLeft);
//      inputMapper.addActionMapping("onMouseButtonRight", new MouseButtonTrigger(MouseInput.BUTTON_RIGHT), this::onMouseButtonRight);
//      inputMapper.addActionMapping("onKeyW", new KeyTrigger(KeyInput.KEY_W), this::onKeyW);
//      inputMapper.addActionMapping("onKeyA", new KeyTrigger(KeyInput.KEY_A), this::onKeyA);
//      inputMapper.addActionMapping("onKeyS", new KeyTrigger(KeyInput.KEY_S), this::onKeyS);
//      inputMapper.addActionMapping("onKeyD", new KeyTrigger(KeyInput.KEY_D), this::onKeyD);
//      inputMapper.addActionMapping("onKeyQ", new KeyTrigger(KeyInput.KEY_Q), this::onKeyQ);
//      inputMapper.addActionMapping("onKeyZ", new KeyTrigger(KeyInput.KEY_Z), this::onKeyZ);
//      inputMapper.build();

      keyListener = new PrivateKeyListener();

      if (addListeners)
      {
         graphics3dAdapter.addKeyListener(keyListener);
         graphics3dAdapter.addMouseListener(this::mouseDragged);
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

   private void updateCameraPose()
   {
      zoom = MathTools.clamp(zoom, 0.1, 100.0);

      latitude = MathTools.clamp(latitude, Math.PI / 2.0);
      longitude = EuclidCoreTools.trimAngleMinusPiToPi(longitude);
      roll = 30.0;

      latitudeAxisAngle.set(Axis3D.X, -latitude);
      longitudeAxisAngle.set(Axis3D.Y, -longitude);
      rollAxisAngle.set(Axis3D.Z, roll);

      focusPointPose.changeFrame(ReferenceFrame.getWorldFrame());
      focusPointPose.setOrientation(longitudeAxisAngle);
      focusPointPose.changeFrame(zUpFrame);

//      focusPointSphere.setLocalTranslation((float) focusPointPose.getX(), (float) focusPointPose.getY(), (float) focusPointPose.getZ());
//      focusPointSphere.setLocalScale((float) (0.0035 * zoom));

      fixPointNode.getTranslation().set(focusPointPose.getPosition());

      cameraPose.setToZero(zUpFrame);
      cameraPose.appendTranslation(focusPointPose.getPosition());
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
      cameraPose.appendRotation(cameraOrientationOffset);
      cameraPose.appendRotation(longitudeAxisAngle);
      cameraPose.appendRotation(latitudeAxisAngle);
      cameraPose.appendRotation(rollAxisAngle);
      cameraPose.appendTranslation(0.0, 0.0, -zoom);

      cameraPose.get(cameraTransform);

      translationJME.set(cameraPose.getPosition().getX32(), cameraPose.getPosition().getY32(), cameraPose.getPosition().getZ32());
      orientationJME.set(cameraPose.getOrientation().getX32(),
                         cameraPose.getOrientation().getY32(),
                         cameraPose.getOrientation().getZ32(),
                         cameraPose.getOrientation().getS32());

//      setFrame(translationJME, orientationJME);
   }

   private void onMouseYUp(float value, float tpf)
   {
      if (leftMousePressed)
      {
         latitude += latitudeSpeed * value;
      }
   }

   private void onMouseYDown(float value, float tpf)
   {
      if (leftMousePressed)
      {
         latitude -= latitudeSpeed * value;
      }
   }

   private void onMouseXLeft(float value, float tpf)
   {
      if (leftMousePressed)
      {
         longitude -= longitudeSpeed * value;
      }
   }

   private void onMouseXRight(float value, float tpf)
   {
      if (leftMousePressed)
      {
         longitude += longitudeSpeed * value;
      }
   }

   private void onMouseScrollUp(float value, float tpf)
   {
      zoom = zoom - zoom * zoomSpeedFactor;
      //      zoom -= zoomSpeedFactor * value;
   }

   private void onMouseScrollDown(float value, float tpf)
   {
      zoom = zoom + zoom * zoomSpeedFactor;
      //      zoom += zoomSpeedFactor * value;
   }

   private void onMouseButtonLeft(boolean isPressed, float tpf)
   {
      leftMousePressed = isPressed;
   }

   private void onMouseButtonRight(boolean isPressed, float tpf)
   {

   }

   private void mouseDragged(MouseButton mouseButton, double dx, double dy)
   {
      switch (mouseButton)
      {
         case LEFT:
            latitude += latitudeSpeed * dy;
            longitude -= longitudeSpeed * dx;
            break;
         case RIGHT:
            break;
         case MIDDLE:
            break;
         case LEFTRIGHT:
            break;
      }
   }

   private void onKeyW(boolean isPressed, float tpf)
   {
      isWPressed = isPressed;
   }

   private void onKeyA(boolean isPressed, float tpf)
   {
      isAPressed = isPressed;
   }

   private void onKeyS(boolean isPressed, float tpf)
   {
      isSPressed = isPressed;
   }

   private void onKeyD(boolean isPressed, float tpf)
   {
      isDPressed = isPressed;
   }

   private void onKeyQ(boolean isPressed, float tpf)
   {
      isQPressed = isPressed;
   }

   private void onKeyZ(boolean isPressed, float tpf)
   {
      isZPressed = isPressed;
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
   public void computeTransform(RigidBodyTransform cameraTransform, float tpf)
   {
      if (isWPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, translateSpeed * tpf);
      }
      if (isAPressed)
      {
         focusPointPose.appendTranslation(translateSpeed * tpf, 0.0, 0.0);
      }
      if (isSPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, -translateSpeed * tpf);
      }
      if (isDPressed)
      {
         focusPointPose.appendTranslation(-translateSpeed * tpf, 0.0, 0.0);
      }
      if (isQPressed)
      {
         focusPointPose.appendTranslation(0.0, translateSpeed * tpf, 0.0);
      }
      if (isZPressed)
      {
         focusPointPose.appendTranslation(0.0, -translateSpeed * tpf, 0.0);
      }

      updateCameraPose();

      cameraTransform.set(this.cameraTransform);
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
