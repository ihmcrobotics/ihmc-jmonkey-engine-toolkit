package us.ihmc.jMonkeyEngineToolkit.camera;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.mouse.MouseButton;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

import javax.swing.*;
import java.util.ArrayList;

public class FocusBasedCameraController implements TrackingDollyCameraController
{
   public static final double MIN_FIELD_OF_VIEW = 0.001;
   public static final double MAX_FIELD_OF_VIEW = 2.0;

   private static final double MIN_CAMERA_POSITION_TO_FIX_DISTANCE = 0.1; // 0.8;

   private double fieldOfView = CameraConfiguration.DEFAULT_FIELD_OF_VIEW;
   private double clipDistanceNear = CameraConfiguration.DEFAULT_CLIP_DISTANCE_NEAR;
   private double clipDistanceFar = CameraConfiguration.DEFAULT_CLIP_DISTANCE_FAR;

   private Point3D cam = new Point3D();
   private Point3D fix = new Point3D();

   private double zoom_factor = 1.0;
   private double rotate_factor = 1.0;
   private double rotate_camera_factor = 1.0;

   private boolean isMounted = false;
   private CameraMountInterface cameraMount;

   private boolean isTracking = true, isTrackingX = true, isTrackingY = true, isTrackingZ = false;
   private boolean isDolly = false, isDollyX = true, isDollyY = true, isDollyZ = false;
   private double trackDX = 0.0, trackDY = 0.0, trackDZ = 0.0;
   private double dollyDX = 2.0, dollyDY = 12.0, dollyDZ = 0.0;

   private ViewportAdapter viewportAdapter;
   private JFrame jFrame;

   // Flying
   private boolean fly = true;
   private boolean forward = false;
   private boolean backward = false;
   private boolean left = false;
   private boolean right = false;
   private boolean up = false;
   private boolean down = false;

   private final FocusBasedCameraReferenceFrame zUpFrame = new FocusBasedCameraReferenceFrame("ZUpFrame", ReferenceFrame.getWorldFrame());

   private final RotationMatrix cameraOrientationOffset = new RotationMatrix();
   private final Vector3D upVector;
   private final Vector3D forwardVector;
   private final Vector3D leftVector;
   private final Vector3D downVector;

   private ArrayList<Point3D> storedCameraPositions = new ArrayList<>(0);
   private ArrayList<Point3D> storedFixPositions = new ArrayList<>(0);
   private int storedPositionIndex = 0;
   private boolean transitioning = false;

   private static double transitionTime = 500;
   private double camXSpeed;
   private double camYSpeed;
   private double camZSpeed;
   private double fixXSpeed;
   private double fixYSpeed;
   private double fixZSpeed;
   private long lastTransitionTime;

   private ArrayList<Point3D> keyFrameCamPos = new ArrayList<>(0);
   private ArrayList<Point3D> keyFrameFixPos = new ArrayList<>(0);
   private ArrayList<Integer> keyFrameTimes = new ArrayList<>(0);

   private Graphics3DNode fixPointNode = new Graphics3DNode("cameraFixPoint",
                                                            new Graphics3DObject(new Sphere3D(0.01), YoAppearance.RGBColor(1.0, 0.0, 0.0, 0.5)));

   private boolean toggleCameraKeyPoints = false;
   private int cameraKeyPointIndex;
   private ArrayList<Integer> cameraKeyPoints = new ArrayList<>(0);

   private CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder;

   private Graphics3DAdapter graphics3dAdapter;

   private KeyListener keyListener;

   public FocusBasedCameraController(Graphics3DAdapter graphics3dAdapter,
                                     ViewportAdapter viewportAdapter,
                                     CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder,
                                     JFrame jFrame,
                                     boolean addListeners)
   {
      if (graphics3dAdapter == null)
         throw new RuntimeException("graphics3dAdapter == null");
      this.graphics3dAdapter = graphics3dAdapter;
      this.viewportAdapter = viewportAdapter;
      this.jFrame = jFrame;

      cam.setX(ClassicCameraController.CAMERA_START_X);
      cam.setY(ClassicCameraController.CAMERA_START_Y);
      cam.setZ(ClassicCameraController.CAMERA_START_Z);
      fix.setX(0.0);
      fix.setY(0.0);
      fix.setZ(0.6);

      RotationMatrix zUpToYUp = new RotationMatrix();
      zUpToYUp.set(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0);
      zUpFrame.setOrientationAndUpdate(zUpToYUp);

      upVector = new Vector3D(0.0, 0.0, 1.0);
      forwardVector = new Vector3D(1.0, 0.0, 0.0);
      leftVector = new Vector3D();
      leftVector.cross(upVector, forwardVector);
      downVector = new Vector3D();
      downVector.setAndNegate(upVector);
      Vector3D cameraZAxis = new Vector3D(forwardVector);
      Vector3D cameraYAxis = new Vector3D(upVector);
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(cameraYAxis, cameraZAxis);
      cameraOrientationOffset.setColumns(cameraXAxis, cameraYAxis, cameraZAxis);

      this.cameraTrackAndDollyVariablesHolder = cameraTrackAndDollyVariablesHolder;

      // Don't do this stuff by default
      setTracking(false);
      setDolly(false);

      keyListener = new PrivateKeyListener();

      if (addListeners)
      {
         graphics3dAdapter.addKeyListener(keyListener);
         graphics3dAdapter.addMouseListener(this::mouseDragged);
         graphics3dAdapter.addMouse3DListener(this::mouseDragged);
         graphics3dAdapter.addSelectedListener(this::selected);
      }
   }

   public void setCameraMount(CameraMountInterface mount)
   {
      cameraMount = mount;
   }

   public CameraMountInterface getCameraMount()
   {
      return cameraMount;
   }

   public boolean isMounted()
   {
      return isMounted;
   }

   @Override
   public CameraTrackingAndDollyPositionHolder getCameraTrackAndDollyVariablesHolder()
   {
      return cameraTrackAndDollyVariablesHolder;
   }

   @Override
   public void setConfiguration(CameraConfiguration config, CameraMountList mountList)
   {
      if (config == null)
         return;

      isMounted = config.isCameraMounted();
      if (isMounted && mountList != null)
      {
         cameraMount = mountList.getCameraMount(config.getCameraMountName());
      }

      cam.setX(config.camX);
      cam.setY(config.camY);
      cam.setZ(config.camZ);
      fix.setX(config.fixX);
      fix.setY(config.fixY);
      fix.setZ(config.fixZ);

      isTracking = config.isTracking;
      isTrackingX = config.isTrackingX;
      isTrackingY = config.isTrackingY;
      isTrackingZ = config.isTrackingZ;
      isDolly = config.isDolly;
      isDollyX = config.isDollyX;
      isDollyY = config.isDollyY;
      isDollyZ = config.isDollyZ;

      trackDX = config.trackDX;
      trackDY = config.trackDY;
      trackDZ = config.trackDZ;
      dollyDX = config.dollyDX;
      dollyDY = config.dollyDY;
      dollyDZ = config.dollyDZ;

      setFieldOfView(config.fieldOfView);
      clipDistanceFar = config.clipDistanceFar;
      clipDistanceNear = config.clipDistanceNear;

      // this.update();
   }

   @Override
   public boolean isTracking()
   {
      return isTracking;
   }

   @Override
   public boolean isTrackingX()
   {
      return isTrackingX;
   }

   @Override
   public boolean isTrackingY()
   {
      return isTrackingY;
   }

   @Override
   public boolean isTrackingZ()
   {
      return isTrackingZ;
   }

   @Override
   public boolean isDolly()
   {
      return isDolly;
   }

   @Override
   public boolean isDollyX()
   {
      return isDollyX;
   }

   @Override
   public boolean isDollyY()
   {
      return isDollyY;
   }

   @Override
   public boolean isDollyZ()
   {
      return isDollyZ;
   }

   @Override
   public void setTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
   {
      setTracking(track);
      setTrackingX(trackX);
      setTrackingY(trackY);
      setTrackingZ(trackZ);
   }

   @Override
   public void setDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
   {
      setDolly(dolly);
      setDollyX(dollyX);
      setDollyY(dollyY);
      setDollyZ(dollyZ);
   }

   @Override
   public void setTrackingOffsets(double dx, double dy, double dz)
   {
      trackDX = dx;
      trackDY = dy;
      trackDZ = dz;
   }

   @Override
   public void setDollyOffsets(double dx, double dy, double dz)
   {
      dollyDX = dx;
      dollyDY = dy;
      dollyDZ = dz;
   }

   @Override
   public void setTracking(boolean track)
   {
      isTracking = track;
   }

   @Override
   public void setTrackingX(boolean trackX)
   {
      isTrackingX = trackX;
   }

   @Override
   public void setTrackingY(boolean trackY)
   {
      isTrackingY = trackY;
   }

   @Override
   public void setTrackingZ(boolean trackZ)
   {
      isTrackingZ = trackZ;
   }

   @Override
   public void setDolly(boolean dolly)
   {
      isDolly = dolly;
   }

   @Override
   public void setDollyX(boolean dollyX)
   {
      isDollyX = dollyX;
   }

   @Override
   public void setDollyY(boolean dollyY)
   {
      isDollyY = dollyY;
   }

   @Override
   public void setDollyZ(boolean dollyZ)
   {
      isDollyZ = dollyZ;
   }

   @Override
   public double getTrackingXOffset()
   {
      return trackDX;
   }

   @Override
   public double getTrackingYOffset()
   {
      return trackDY;
   }

   @Override
   public double getTrackingZOffset()
   {
      return trackDZ;
   }

   @Override
   public double getDollyXOffset()
   {
      return dollyDX;
   }

   @Override
   public double getDollyYOffset()
   {
      return dollyDY;
   }

   @Override
   public double getDollyZOffset()
   {
      return dollyDZ;
   }

   @Override
   public void setTrackingXOffset(double dx)
   {
      trackDX = dx;
   }

   @Override
   public void setTrackingYOffset(double dy)
   {
      trackDY = dy;
   }

   @Override
   public void setTrackingZOffset(double dz)
   {
      trackDZ = dz;
   }

   @Override
   public void setDollyXOffset(double dx)
   {
      dollyDX = dx;
   }

   @Override
   public void setDollyYOffset(double dy)
   {
      dollyDY = dy;
   }

   @Override
   public void setDollyZOffset(double dz)
   {
      dollyDZ = dz;
   }

   @Override
   public void update()
   {
      if (graphics3dAdapter.getContextManager().getCurrentViewport() != viewportAdapter)
      {
         forward = false;
         backward = false;
         left = false;
         right = false;
         up = false;
         down = false;
      }

      if (isTracking)
      {
         if (isTrackingX)
         {
            double trackX = cameraTrackAndDollyVariablesHolder.getTrackingX();
            if (!Double.isNaN(trackX))
               fix.setX(trackX + trackDX);
         }

         if (isTrackingY)
         {
            double trackY = cameraTrackAndDollyVariablesHolder.getTrackingY();
            if (!Double.isNaN(trackY))
               fix.setY(trackY + trackDY);
         }

         if (isTrackingZ)
         {
            double trackZ = cameraTrackAndDollyVariablesHolder.getTrackingZ();
            if (!Double.isNaN(trackZ))
               fix.setZ(trackZ + trackDZ);
         }
      }

      if (isDolly)
      {
         double dollyX = cameraTrackAndDollyVariablesHolder.getDollyX();
         if (isDollyX)
         {
            if (!Double.isNaN(dollyX))
               cam.setX(dollyX + dollyDX);
         }

         if (isDollyY)
         {
            double dollyY = cameraTrackAndDollyVariablesHolder.getDollyY();
            if (!Double.isNaN(dollyY))
               cam.setY(dollyY + dollyDY);
         }

         if (isDollyZ)
         {
            double dollyZ = cameraTrackAndDollyVariablesHolder.getDollyZ();
            if (!Double.isNaN(dollyZ))
               cam.setZ(dollyZ + dollyDZ);
         }
      }

      double fieldOfView = cameraTrackAndDollyVariablesHolder.getFieldOfView();
      if (!Double.isNaN(fieldOfView))
         setFieldOfView(fieldOfView);

      // Flying
      if (fly && !isTracking && !isDolly && !transitioning)
      {
         if (forward)
         {
            moveCameraForward(-0.5);
         }

         if (backward)
         {
            moveCameraForward(0.5);
         }

         if (left)
         {
            pan(20, 0);
         }

         if (right)
         {
            pan(-20, 0);
         }

         if (up)
         {
            pan(00, 20);
         }

         if (down)
         {
            pan(00, -20);
         }
      }

      // End Flying

      if (transitioning && !isTracking && !isDolly)
      {
         int numberOfDimensionsThatHaveTransitioned = 0;
         double elapsedTransitionTime = System.currentTimeMillis() - lastTransitionTime;
         lastTransitionTime = System.currentTimeMillis();

         if (Math.abs(cam.getX() - storedCameraPositions.get(storedPositionIndex).getX()) <= Math.abs(camXSpeed * elapsedTransitionTime))
         {
            cam.setX(storedCameraPositions.get(storedPositionIndex).getX());
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            cam.addX(camXSpeed * elapsedTransitionTime);
         }

         if (Math.abs(cam.getY() - storedCameraPositions.get(storedPositionIndex).getY()) <= Math.abs(camYSpeed * elapsedTransitionTime))
         {
            cam.setY(storedCameraPositions.get(storedPositionIndex).getY());
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            cam.addY(camYSpeed * elapsedTransitionTime);
         }

         if (Math.abs(cam.getZ() - storedCameraPositions.get(storedPositionIndex).getZ()) <= Math.abs(camZSpeed * elapsedTransitionTime))
         {
            cam.setZ(storedCameraPositions.get(storedPositionIndex).getZ());
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            cam.addZ(camZSpeed * elapsedTransitionTime);
         }

         if (Math.abs(fix.getX() - storedFixPositions.get(storedPositionIndex).getX()) <= Math.abs(fixXSpeed * elapsedTransitionTime))
         {
            fix.setX(storedFixPositions.get(storedPositionIndex).getX());
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            fix.addX(fixXSpeed * elapsedTransitionTime);
         }

         if (Math.abs(fix.getY() - storedFixPositions.get(storedPositionIndex).getY()) <= Math.abs(fixYSpeed * elapsedTransitionTime))
         {
            fix.setY(storedFixPositions.get(storedPositionIndex).getY());
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            fix.addY(fixYSpeed * elapsedTransitionTime);
         }

         if (Math.abs(fix.getZ() - storedFixPositions.get(storedPositionIndex).getZ()) <= Math.abs(fixZSpeed * elapsedTransitionTime))
         {
            fix.setZ(storedFixPositions.get(storedPositionIndex).getZ());
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            fix.addZ(fixZSpeed * elapsedTransitionTime);
         }

         if (numberOfDimensionsThatHaveTransitioned == 6)
         {
            transitioning = false;
         }
      }
   }

   public void addKeyFrame(int time)
   {
      addKeyFrame(keyFrameCamPos.size(), time);
   }

   public void addKeyFrame(int i, int time)
   {
      keyFrameCamPos.add(i, new Point3D(cam.getX(), cam.getY(), cam.getZ()));
      keyFrameFixPos.add(i, new Point3D(fix.getX(), fix.getY(), fix.getZ()));
      keyFrameTimes.add(i, time);
   }

   public int removeKeyFrameByTime(int time)
   {
      for (int i = 0; i < keyFrameTimes.size(); i++)
      {
         if (keyFrameTimes.get(i) == time)
         {
            removeKeyFrameByIndex(i);

            return i;
         }
      }

      return -1;
   }

   public void removeKeyFrameByIndex(int i)
   {
      if (i >= 0 && i < keyFrameTimes.size())
      {
         keyFrameTimes.remove(i);
         keyFrameCamPos.remove(i);
         keyFrameFixPos.remove(i);
      }
   }

   @Override
   public void setKeyFrameTime(int time)
   {
      for (int i = keyFrameTimes.size() - 1; i >= 0; i--)
      {
         if (time >= keyFrameTimes.get(i))
         {
            if (keyFrameTimes.size() > i + 1)
            {
               double elapsedTime = time - keyFrameTimes.get(i);
               double totalTime = keyFrameTimes.get(i + 1) - keyFrameTimes.get(i);
               cam.setX(keyFrameCamPos.get(i).getX() + (keyFrameCamPos.get(i + 1).getX() - keyFrameCamPos.get(i).getX()) * elapsedTime / totalTime);
               cam.setY(keyFrameCamPos.get(i).getY() + (keyFrameCamPos.get(i + 1).getY() - keyFrameCamPos.get(i).getY()) * elapsedTime / totalTime);
               cam.setZ(keyFrameCamPos.get(i).getZ() + (keyFrameCamPos.get(i + 1).getZ() - keyFrameCamPos.get(i).getZ()) * elapsedTime / totalTime);

               fix.setX(keyFrameFixPos.get(i).getX() + (keyFrameFixPos.get(i + 1).getX() - keyFrameFixPos.get(i).getX()) * elapsedTime / totalTime);
               fix.setY(keyFrameFixPos.get(i).getY() + (keyFrameFixPos.get(i + 1).getY() - keyFrameFixPos.get(i).getY()) * elapsedTime / totalTime);
               fix.setZ(keyFrameFixPos.get(i).getZ() + (keyFrameFixPos.get(i + 1).getZ() - keyFrameFixPos.get(i).getZ()) * elapsedTime / totalTime);
            }

            break;
         }
      }
   }

   public void gotoKey(int index)
   {
      if (index >= 0 && index < keyFrameCamPos.size())
      {
         storedPositionIndex = index;
         cam.setX(keyFrameCamPos.get(index).getX());
         cam.setY(keyFrameCamPos.get(index).getY());
         cam.setZ(keyFrameCamPos.get(index).getZ());

         fix.setX(keyFrameFixPos.get(index).getX());
         fix.setY(keyFrameFixPos.get(index).getY());
         fix.setZ(keyFrameFixPos.get(index).getZ());
      }
   }

   @Override
   public ArrayList<Integer> getCameraKeyPoints()
   {
      return cameraKeyPoints;
   }

   @Override
   public double getFixX()
   {
      return fix.getX();
   }

   @Override
   public double getFixY()
   {
      return fix.getY();
   }

   @Override
   public double getFixZ()
   {
      return fix.getZ();
   }

   @Override
   public double getCamX()
   {
      return cam.getX();
   }

   @Override
   public double getCamY()
   {
      return cam.getY();
   }

   @Override
   public double getCamZ()
   {
      return cam.getZ();
   }

   @Override
   public void setFixX(double fx)
   {
      fix.setX(fx);
   }

   @Override
   public void setFixY(double fy)
   {
      fix.setY(fy);
   }

   @Override
   public void setFixZ(double fz)
   {
      fix.setZ(fz);
   }

   @Override
   public void setCamX(double cx)
   {
      cam.setX(cx);
   }

   @Override
   public void setCamY(double cy)
   {
      cam.setY(cy);
   }

   @Override
   public void setCamZ(double cz)
   {
      cam.setZ(cz);
   }

   @Override
   public void setFixPosition(double fx, double fy, double fz)
   {
      fix.setX(fx);
      fix.setY(fy);
      fix.setZ(fz);
   }

   @Override
   public void setCameraPosition(double cx, double cy, double cz)
   {
      cam.setX(cx);
      cam.setY(cy);
      cam.setZ(cz);
   }

   private Vector3D v3d = new Vector3D();
   private RigidBodyTransform t3d = new RigidBodyTransform();
   private Vector3D rotVector = new Vector3D();
   private AxisAngle rotAxisAngle4d = new AxisAngle();

   public void doMouseDraggedLeft(double dx, double dy)
   {
      // Rotate around fix point:

      double delX0 = cam.getX() - fix.getX(), delY0 = cam.getY() - fix.getY(), delZ0 = cam.getZ() - fix.getZ();
      v3d.set(delX0, delY0, delZ0);

      // double offsetDistance = v3d.length();

      t3d.setRotationYawAndZeroTranslation(-dx * rotate_factor);
      t3d.transform(v3d);

      if (!isDolly || !isDollyX && !isDollyY)
      {
         cam.setX(v3d.getX() + fix.getX());
         cam.setY(v3d.getY() + fix.getY());
      }

      delX0 = cam.getX() - fix.getX();
      delY0 = cam.getY() - fix.getY();
      delZ0 = cam.getZ() - fix.getZ();

      // v3d.set(delX0, delY0, delZ0);
      rotVector.cross(new Vector3D(0.0, 0.0, -1.0), v3d);
      rotAxisAngle4d.set(rotVector, dy * rotate_factor / 4.0);

      t3d.setRotationAndZeroTranslation(rotAxisAngle4d);
      t3d.transform(v3d);

      if (v3d.getX() * delX0 > 0.0 && v3d.getY() * delY0 > 0.0)
      {
         if (!isDolly || !isDollyX && !isDollyY)
         {
            cam.setX(v3d.getX() + fix.getX());
            cam.setY(v3d.getY() + fix.getY());
         }

         if (!isDolly || !isDollyZ)
         {
            cam.setZ(v3d.getZ() + fix.getZ());

            /*
             * double factor = elevate_factor * Math.abs(offsetDistance); //cam.getZ()-fix.getZ()); if (factor <
             * elevate_factor) factor = elevate_factor; //cam.setZ(v3d.z + fix.getZ() + dy * elevate_factor; camZ =
             * v3d.z + fix.getZ() + dy * factor;
             */
         }
      }

      // transformChanged(currXform);
   }

   public void rotateAroundFix(double dx, double dy)
   {
      double distanceFromCameraToFix = Math.sqrt(Math.pow(cam.getX() - fix.getX(), 2) + Math.pow(cam.getY() - fix.getY(), 2) + Math.pow(cam.getZ() - fix.getZ(), 2));

      if (distanceFromCameraToFix > 1.0)
      {
         dx /= distanceFromCameraToFix;
         dy /= distanceFromCameraToFix;
      }

      double delX0 = cam.getX() - fix.getX(), delY0 = cam.getY() - fix.getY(), delZ0 = cam.getZ() - fix.getZ();
      v3d.set(delX0, delY0, delZ0);

      t3d.setRotationYawAndZeroTranslation(-dx * rotate_factor);
      t3d.transform(v3d);

      if (!isDolly || !isDollyX && !isDollyY)
      {
         cam.setX(v3d.getX() + fix.getX());
         cam.setY(v3d.getY() + fix.getY());
      }

      delX0 = cam.getX() - fix.getX();
      delY0 = cam.getY() - fix.getY();
      delZ0 = cam.getZ() - fix.getZ();

      rotVector.cross(new Vector3D(0.0, 0.0, -1.0), v3d);
      rotAxisAngle4d.set(rotVector, dy * rotate_factor / 4.0);

      t3d.setRotationAndZeroTranslation(rotAxisAngle4d);
      t3d.transform(v3d);

      if (v3d.getX() * delX0 > 0.0 && v3d.getY() * delY0 > 0.0)
      {
         if (!isDolly || !isDollyX && !isDollyY)
         {
            cam.setX(v3d.getX() + fix.getX());
            cam.setY(v3d.getY() + fix.getY());
         }

         if (!isDolly || !isDollyZ)
         {
            cam.setZ(v3d.getZ() + fix.getZ());
         }
      }
   }

   public void doMouseDraggedRight(double dx, double dy)
   {
      // Elevate up and down
      double delX0 = cam.getX() - fix.getX(), delY0 = cam.getY() - fix.getY(), delZ0 = cam.getZ() - fix.getZ();
      v3d.set(delX0, delY0, delZ0);

      // double offsetDistance = v3d.length();

      t3d.setRotationYawAndZeroTranslation(-dx * rotate_camera_factor);
      t3d.transform(v3d);

      if (!isTracking || !isTrackingX && !isTrackingY)
      {
         fix.setX(cam.getX() - v3d.getX());
         fix.setY(cam.getY() - v3d.getY());
      }

      delX0 = cam.getX() - fix.getX();
      delY0 = cam.getY() - fix.getY();
      delZ0 = cam.getZ() - fix.getZ();

      // v3d.set(delX0, delY0, delZ0);

      rotVector.set(-1.0, 0.0, 0.0);
      rotVector.cross(new Vector3D(0.0, 0.0, -1.0), v3d);
      rotAxisAngle4d.set(rotVector, dy * rotate_camera_factor / 4.0);

      t3d.setRotationAndZeroTranslation(rotAxisAngle4d);
      t3d.transform(v3d);

      if (v3d.getX() * delX0 > 0.0 && v3d.getY() * delY0 > 0.0)
      {
         if (!isTracking || !isTrackingX && !isTrackingY)
         {
            fix.setX(cam.getX() - v3d.getX());
            fix.setY(cam.getY() - v3d.getY());
         }

         if (!isTracking || !isTrackingZ)
         {
            fix.setZ(cam.getZ() - v3d.getZ());

            /*
             * double factor = elevate_camera_factor * offsetDistance; //Math.abs(cam.getZ()-fix.getZ()); if (factor <
             * elevate_camera_factor) factor = elevate_camera_factor; fix.getZ() = cam.getZ() - v3d.z + dy factor; //fix.getZ() =
             * cam.getZ() - v3d.z + dy * elevate_factor;
             */
         }
      }

      // transformChanged(currXform);
   }

   @Override
   public void setFieldOfView(double fov)
   {
      fieldOfView = fov;

      if (fieldOfView < MIN_FIELD_OF_VIEW)
         fieldOfView = MIN_FIELD_OF_VIEW;
      if (fieldOfView > MAX_FIELD_OF_VIEW)
         fieldOfView = MAX_FIELD_OF_VIEW;
   }

   public void doMouseDraggedMiddle(double dx, double dy)
   {
      // Zooms in and out

      if (isMounted && viewportAdapter != null)
      {
         cameraMount.zoom(dy * 0.1);
      }

      else
      {
         Vector3D v3d = new Vector3D(cam.getX() - fix.getX(), cam.getY() - fix.getY(), cam.getZ() - fix.getZ());

         Vector3D offsetVec = new Vector3D(v3d);

         // offsetVec.normalize();
         offsetVec.scale(dy * zoom_factor);

         // if (offsetVec.length() < v3d.length())
         // {
         if (!isDolly || !isDollyX && !isDollyY)
         {
            cam.addX(offsetVec.getX());
            cam.addY(offsetVec.getY());
         }

         if (!isDolly || !isDollyZ)
            cam.addZ(offsetVec.getZ());

         // }

         v3d.set(cam.getX() - fix.getX(), cam.getY() - fix.getY(), cam.getZ() - fix.getZ());

         if (v3d.length() < MIN_CAMERA_POSITION_TO_FIX_DISTANCE)
         {
            v3d.normalize();
            v3d.scale(MIN_CAMERA_POSITION_TO_FIX_DISTANCE);
            cam.setX(v3d.getX() + fix.getX());
            cam.setY(v3d.getY() + fix.getY());
            cam.setZ(v3d.getZ() + fix.getZ());
         }
      }

      // transformChanged(currXform);
   }

   private void moveCameraForward(double distance)
   {
      double angleXY = Math.atan2(cam.getY() - fix.getY(), cam.getX() - fix.getX());
      double angleZ = Math.atan2(cam.getZ() - fix.getZ(), Math.hypot(cam.getY() - fix.getY(), cam.getX() - fix.getX()));

      double distXY = distance * Math.cos(angleZ);

      cam.addX(distXY * Math.cos(angleXY));
      cam.addY(distXY * Math.sin(angleXY));
      cam.addZ(distance * Math.sin(angleZ));

      if (Math.sqrt(Math.pow(cam.getX() - fix.getX(), 2) + Math.pow(cam.getY() - fix.getY(), 2) + Math.pow(cam.getY() - fix.getY(), 2)) < 1)
      {
         fix.addX(distXY * Math.cos(angleXY));
         fix.addY(distXY * Math.sin(angleXY));
         fix.addZ(distance * Math.sin(angleZ));
      }

      // Vector3d v3d = new Vector3d(cam.getX() - fix.getX(), cam.getY() - fix.getY(), cam.getZ() - fix.getZ());
      //
      // Vector3d offsetVec = new Vector3d(v3d);
      //
      //// offsetVec.normalize();
      // offsetVec.scale(distance * zoom_factor);
      //
      //// if (offsetVec.length() < v3d.length())
      //// {
      // if (!isDolly || (!isDollyX &&!isDollyY))
      // {
      // cam.getX() += offsetVec.x;
      // cam.addY(offsetVec.y;
      // }
      //
      // if (!isDolly ||!isDollyZ)
      // cam.addZ(offsetVec.z;
   }

   public void pan(double dx, double dy)
   {
      double distanceFromCameraToFix = Math.sqrt(Math.pow(cam.getX() - fix.getX(), 2) + Math.pow(cam.getY() - fix.getY(), 2) + Math.pow(cam.getZ() - fix.getZ(), 2));
      dx *= distanceFromCameraToFix / viewportAdapter.getPhysicalWidth() * .00023;
      dy *= distanceFromCameraToFix / viewportAdapter.getPhysicalHeight() * .00007;
      double theta = Math.PI / 2 + Math.atan2(cam.getZ() - fix.getZ(), Math.hypot(cam.getX() - fix.getX(), cam.getY() - fix.getY()));
      if (!isTracking || !isTrackingZ)
      {
         cam.addZ(dy * Math.sin(theta));
         fix.addZ(dy * Math.sin(theta));
      }

      double d = dy * Math.cos(theta);
      theta = Math.atan2(cam.getY() - fix.getY(), cam.getX() - fix.getX());

      if (!isTracking || !isTrackingY)
      {
         cam.addY(d * Math.sin(theta));
         fix.addY(d * Math.sin(theta));
      }

      if (!isTracking || !isTrackingX)
      {
         cam.addX(d * Math.cos(theta));
         fix.addX(d * Math.cos(theta));
      }

      theta = Math.PI / 2 + Math.atan2(cam.getY() - fix.getY(), cam.getX() - fix.getX());

      if (!isTracking || !isTrackingY)
      {
         cam.subY(dx * Math.sin(theta));
         fix.subY(dx * Math.sin(theta));
      }

      if (!isTracking || !isTrackingX)
      {
         cam.subX(dx * Math.cos(theta));
         fix.subX(dx * Math.cos(theta));
      }
   }

   public void translateFix(double dx, double dy, double dz)
   {
      //      double zTiltAngle = Math.PI / 2 + Math.atan2(cam.getZ() - fix.getZ(), Math.hypot(cam.getX() - fix.getX(), cam.getY() - fix.getY()));

      //      double ky = dy * Math.sin(zTiltAngle);
      //      double kz = dz * Math.cos(zTiltAngle);

      double yTiltAngle = Math.atan2(cam.getY() - fix.getY(), cam.getX() - fix.getX());

      if (!isTracking || !isTrackingZ)
      {
         cam.addZ(dz);
         fix.addZ(dz);
      }

      if (!isTracking || !isTrackingY)
      {
         cam.addY(dy * Math.sin(yTiltAngle));
         fix.addY(dy * Math.sin(yTiltAngle));
      }

      if (!isTracking || !isTrackingX)
      {
         cam.addX(dy * Math.cos(yTiltAngle));
         fix.addX(dy * Math.cos(yTiltAngle));
      }

      double xTiltAngle = yTiltAngle + Math.PI / 2;

      if (!isTracking || !isTrackingY)
      {
         cam.subY(dx * Math.sin(xTiltAngle));
         fix.subY(dx * Math.sin(xTiltAngle));
      }

      if (!isTracking || !isTrackingX)
      {
         cam.subX(dx * Math.cos(xTiltAngle));
         fix.subX(dx * Math.cos(xTiltAngle));
      }
   }

   private void initTransition()
   {
      camXSpeed = -(cam.getX() - storedCameraPositions.get(storedPositionIndex).getX()) / transitionTime;
      camYSpeed = -(cam.getY() - storedCameraPositions.get(storedPositionIndex).getY()) / transitionTime;
      camZSpeed = -(cam.getZ() - storedCameraPositions.get(storedPositionIndex).getZ()) / transitionTime;

      fixXSpeed = -(fix.getX() - storedFixPositions.get(storedPositionIndex).getX()) / transitionTime;
      fixYSpeed = -(fix.getY() - storedFixPositions.get(storedPositionIndex).getY()) / transitionTime;
      fixZSpeed = -(fix.getZ() - storedFixPositions.get(storedPositionIndex).getZ()) / transitionTime;

      transitioning = true;
      lastTransitionTime = System.currentTimeMillis();
   }

   @Override
   public void toggleCameraKeyMode()
   {
      setUseCameraKeyPoints(!useKeyCameraPoints());
   }

   @Override
   public boolean getCameraKeyMode()
   {
      return toggleCameraKeyPoints;
   }

   @Override
   public void setUseCameraKeyPoints(boolean use)
   {
      toggleCameraKeyPoints = use;
   }

   @Override
   public boolean useKeyCameraPoints()
   {
      return toggleCameraKeyPoints;
   }

   @Override
   public boolean setCameraKeyPoint(int time)
   {
      boolean added = false;
      for (int i = 0; i < cameraKeyPoints.size(); i++)
      {
         if (cameraKeyPoints.get(i) == time)
         {
            addKeyFrame(removeKeyFrameByTime(time), time);

            return false;
         }

         if (cameraKeyPoints.get(i) > time)
         {
            cameraKeyPoints.add(time);
            addKeyFrame(1, time);

            return true;
         }
      }

      if (!added)
      {
         cameraKeyPoints.add(time);
         addKeyFrame(time);
      }

      return true;
   }

   @Override
   public void nextCameraKeyPoint(int time)
   {
      // int closestLesserTime = Integer.MIN_VALUE;
      // int index = -1;
      // for (int i = 0; i < cameraKeyPoints.size(); i++)
      // {
      // if (cameraKeyPoints.get(i) < time && cameraKeyPoints.get(i) > closestLesserTime)
      // {
      // index = i;
      // closestLesserTime = cameraKeyPoints.get(i);
      // }
      // }
      // if (index != -1)
      // {
      // camera.initTransition(index);
      // }
      cameraKeyPointIndex++;

      if (cameraKeyPointIndex >= cameraKeyPoints.size())
      {
         cameraKeyPointIndex = 0;
      }

      toggleCameraKeyPoints = false;
      gotoKey(cameraKeyPointIndex);
   }

   @Override
   public void previousCameraKeyPoint(int time)
   {
      cameraKeyPointIndex--;

      if (cameraKeyPointIndex < 0)
      {
         cameraKeyPointIndex = cameraKeyPoints.size() - 1;
      }

      toggleCameraKeyPoints = false;
      gotoKey(cameraKeyPointIndex);
   }

   @Override
   public void removeCameraKeyPoint(int time)
   {
      cameraKeyPoints.remove(cameraKeyPointIndex);
      removeKeyFrameByIndex(cameraKeyPointIndex);
   }

   @Override
   public double getTrackXVar()
   {
      return cameraTrackAndDollyVariablesHolder.getTrackingX();
   }

   @Override
   public double getTrackYVar()
   {
      return cameraTrackAndDollyVariablesHolder.getTrackingY();
   }

   @Override
   public double getTrackZVar()
   {
      return cameraTrackAndDollyVariablesHolder.getTrackingZ();
   }

   @Override
   public double getDollyXVar()
   {
      return cameraTrackAndDollyVariablesHolder.getDollyX();
   }

   @Override
   public double getDollyYVar()
   {
      return cameraTrackAndDollyVariablesHolder.getDollyY();
   }

   @Override
   public double getDollyZVar()
   {
      return cameraTrackAndDollyVariablesHolder.getDollyZ();
   }

   public void nextStoredPosition()
   {
      if (storedCameraPositions.size() > 0)
      {
         storedPositionIndex++;

         if (storedPositionIndex >= storedCameraPositions.size())
         {
            storedPositionIndex = 0;
         }

         initTransition();
      }
   }

   public void previousStoredPosition()
   {
      if (storedCameraPositions.size() > 0)
      {
         storedPositionIndex--;

         if (storedPositionIndex < 0)
         {
            storedPositionIndex = storedCameraPositions.size() - 1;
         }

         initTransition();
      }
   }

   public void storePosition()
   {
      storedCameraPositions.add(new Point3D(getCamX(), getCamY(), getCamZ()));
      storedFixPositions.add(new Point3D(getFixX(), getFixY(), getFixZ()));
   }

   @Override
   public void reset()
   {

   }

   private RotationMatrix rotationMatrix = new RotationMatrix();
   private Vector3D positionOffset = new Vector3D();

   private Vector3D zAxis = new Vector3D(), yAxis = new Vector3D(), xAxis = new Vector3D();

   @Override
   public void computeTransform(RigidBodyTransform currXform)
   {
      update();
      CameraMountInterface cameraMount = getCameraMount();
      if (isMounted() && cameraMount != null)
      {
         cameraMount.getTransformToCamera(currXform);

         return;
      }

      positionOffset.set(getCamX(), getCamY(), getCamZ());
      xAxis.set(getFixX(), getFixY(), getFixZ());

      fixPointNode.translateTo(getFixX(), getFixY(), getFixZ());

      xAxis.sub(positionOffset);
      xAxis.normalize();
      zAxis.set(0.0, 0.0, 1.0);
      yAxis.cross(zAxis, xAxis);
      yAxis.normalize();
      zAxis.cross(xAxis, yAxis);

      rotationMatrix.setColumns(xAxis, yAxis, zAxis);

      currXform.setRotationAndZeroTranslation(rotationMatrix);
      currXform.getTranslation().set(positionOffset);
      currXform.getRotation().normalize();
   }

   private boolean shouldAcceptDeviceInput()
   {
      if (alreadyClosing || graphics3dAdapter.getContextManager().getCurrentViewport() != viewportAdapter)
         return false;

      if (jFrame != null && !jFrame.isActive())
         return false;

      return true;
   }

   class PrivateKeyListener implements KeyListener
   {
      @Override
      public void keyPressed(Key key)
      {
         if (shouldAcceptDeviceInput())
         {
            switch (key)
            {
               case W:
                  forward = true;

                  break;

               case S:
                  backward = true;

                  break;

               case A:
                  left = true;

                  break;

               case D:
                  right = true;

                  break;

               case Q:
                  up = true;

                  break;

               case Z:
                  down = true;

                  break;

               default:

                  break;
            }
         }
      }

      @Override
      public void keyReleased(Key key)
      {
         if (shouldAcceptDeviceInput())
         {
            switch (key)
            {
               case W:
                  forward = false;

                  break;

               case S:
                  backward = false;

                  break;

               case A:
                  left = false;

                  break;

               case D:
                  right = false;

                  break;

               case Q:
                  up = false;

                  break;

               case Z:
                  down = false;

                  break;

               case RIGHT:
                  nextStoredPosition();

                  break;

               case LEFT:
                  previousStoredPosition();

                  break;

               case K:
                  storePosition();

                  break;
               default:

                  break;
            }
         }
      }
   }

   private void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location, Point3DReadOnly cameraLocation,
                        QuaternionReadOnly cameraRotation)
   {
      if (shouldAcceptDeviceInput())
      {
         if (modifierKeyInterface.isKeyPressed(Key.SHIFT))
         {
            if (!isTracking() || !isTrackingX())
               setFixX(location.getX());
            if (!isTracking() || !isTrackingY())
               setFixY(location.getY());
            if (!isTracking() || !isTrackingZ())
               setFixZ(location.getZ());
         }
      }
   }

   private void mouseDragged(MouseButton mouseButton, double dx, double dy)
   {
      if (shouldAcceptDeviceInput())
      {
         switch (mouseButton)
         {
            case LEFT:
               doMouseDraggedLeft(dx, dy);

               break;

            case RIGHT:
               doMouseDraggedRight(dx, dy);

               break;

            case MIDDLE:
               doMouseDraggedMiddle(dx, dy);

               break;

            case LEFTRIGHT:
               pan(dx, dy);

               break;
         }
      }
   }

   private double rotateGain = 0.15;
   private double translateGain = 0.05;

   private void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
   {
      if (shouldAcceptDeviceInput())
      {
         //      doMouseDraggedRight(drz, drx);
         //      doMouseDraggedMiddle(0.0, dz);
         //      moveCameraForward(dy);

         rotateAroundFix(-drz * rotateGain, -drx * rotateGain);
         translateFix(-dx * translateGain, -dy * translateGain, dz * translateGain);
      }
   }

   @Override
   public double getClipNear()
   {
      if (isMounted)
      {
         return cameraMount.getClipDistanceNear();
      }
      else
      {
         return clipDistanceNear;
      }
   }

   @Override
   public double getClipFar()
   {
      if (isMounted)
      {
         return cameraMount.getClipDistanceFar();
      }
      else
      {
         return clipDistanceFar;
      }
   }

   @Override
   public double getHorizontalFieldOfViewInRadians()
   {
      if (isMounted)
      {
         return cameraMount.getFieldOfView();
      }
      else
      {
         return fieldOfView;
      }
   }

   @Override
   public void setClipDistanceNear(double near)
   {
      clipDistanceNear = near;
   }

   @Override
   public void setClipDistanceFar(double far)
   {
      clipDistanceFar = far;
   }

   @Override
   public void copyPositionTrackingDollyConfiguration(TrackingDollyCameraController otherCamera)
   {
      setTracking(otherCamera.isTracking(), otherCamera.isTrackingX(), otherCamera.isTrackingY(), otherCamera.isTrackingZ());
      setDolly(otherCamera.isDolly(), otherCamera.isDollyX(), otherCamera.isDollyY(), otherCamera.isDollyZ());

      setCameraPosition(otherCamera.getCamX(), otherCamera.getCamY(), otherCamera.getCamZ());

      setFixPosition(otherCamera.getFixX(), otherCamera.getFixY(), otherCamera.getFixZ());

      setDollyOffsets(otherCamera.getDollyXOffset(), otherCamera.getDollyYOffset(), otherCamera.getDollyZOffset());
      setTrackingOffsets(otherCamera.getTrackingXOffset(), otherCamera.getTrackingYOffset(), otherCamera.getTrackingZOffset());

      if (otherCamera instanceof FocusBasedCameraController)
      {
         FocusBasedCameraController classicOtherCamera = (FocusBasedCameraController) otherCamera;

         keyFrameCamPos = classicOtherCamera.keyFrameCamPos;
         keyFrameFixPos = classicOtherCamera.keyFrameFixPos;
         keyFrameTimes = classicOtherCamera.keyFrameTimes;

         toggleCameraKeyPoints = classicOtherCamera.toggleCameraKeyPoints;
         cameraKeyPointIndex = classicOtherCamera.cameraKeyPointIndex;
         cameraKeyPoints = classicOtherCamera.cameraKeyPoints;

         System.out.println("Copying camera keys");
      }
   }

   private boolean alreadyClosing = false;

   @Override
   public void closeAndDispose()
   {
      if (alreadyClosing)
         return;
      alreadyClosing = true;

      cameraMount = null;
      viewportAdapter = null;
      if (cameraTrackAndDollyVariablesHolder != null)
      {
         cameraTrackAndDollyVariablesHolder.closeAndDispose();
         cameraTrackAndDollyVariablesHolder = null;
      }
      graphics3dAdapter = null;
   }

   @Override
   public Graphics3DNode getFixPointNode()
   {
      return fixPointNode;
   }
}
