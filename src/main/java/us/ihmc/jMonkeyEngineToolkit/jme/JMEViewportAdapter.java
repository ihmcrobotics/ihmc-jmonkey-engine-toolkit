package us.ihmc.jMonkeyEngineToolkit.jme;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Toolkit;
import java.util.ArrayList;

import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.light.DirectionalLight;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.SceneProcessor;
import com.jme3.profile.AppProfiler;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.JmeContext;
import com.jme3.system.awt.AwtPanel;
import com.jme3.system.awt.AwtPanelsContext;
import com.jme3.system.awt.PaintMode;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jMonkeyEngineToolkit.ContextSwitchedListener;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DFrameListener;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanel;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanelsContext;
import us.ihmc.jMonkeyEngineToolkit.jme.input.JMEInputManager;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;

public class JMEViewportAdapter extends ViewportAdapter implements InputMapSetter, SceneProcessor
{
   public enum ViewportType
   {
      OFFSCREEN, CANVAS, MULTICAM
   }

   private JMECamera jmeCamera;
   private JMEInputManager jmeInputManager;
   private final boolean isMainViewport;

   private JmeContext context;

   private Canvas panel = null;
   private FrameBuffer frameBuffer = null;
   private ViewportType viewportType;
   private ViewPort viewPort;
   private AssetManager assetManager;
   private ArrayList<ContextSwitchedListener> contextSwitchedListeners = new ArrayList<>();

   private RenderManager renderManager;

   private JMEFastCaptureDevice screenShotHelper;
   private AppStateManager stateManager;

   private DirectionalLight primaryLight;
   private FilterPostProcessor fpp;

   private final JMERenderer jmeRenderer;

   private Stopwatch frameTimer = new Stopwatch().start();

   public JMEViewportAdapter(JMERenderer jmeRenderer, Node rootNode, boolean isMainViewport, ViewportType viewportType)
   {
      this(jmeRenderer, rootNode, isMainViewport, viewportType, false, Color.LIGHT_GRAY);
   }

   public JMEViewportAdapter(JMERenderer jmeRenderer, Node rootNode, boolean isMainViewport, ViewportType viewportType, boolean addExtraVisuals)
   {
      this(jmeRenderer, rootNode, isMainViewport, viewportType, addExtraVisuals, Color.LIGHT_GRAY);
   }

   public JMEViewportAdapter(JMERenderer jmeRenderer, Node rootNode, boolean isMainViewport, ViewportType viewportType, boolean addExtraVisuals,
                             Color backgroundColor)
   {
      this(jmeRenderer, rootNode, isMainViewport, viewportType, addExtraVisuals, backgroundColor, true);
   }

   public JMEViewportAdapter(JMERenderer jmeRenderer, Node rootNode, boolean isMainViewport, ViewportType viewportType, boolean addExtraVisuals,
                             Color backgroundColor, boolean flipY)
   {
      this.jmeRenderer = jmeRenderer;
      assetManager = jmeRenderer.getAssetManager();
      stateManager = jmeRenderer.getStateManager();
      context = jmeRenderer.getContext();
      primaryLight = jmeRenderer.getPrimaryLight();
      this.isMainViewport = isMainViewport;
      renderManager = jmeRenderer.getRenderManager();
      jmeCamera = new JMECamera(jmeRenderer.getCamera());
      viewPort = renderManager.createMainView("JMEViewport", jmeCamera);
      viewPort.attachScene(rootNode);
      viewPort.setClearFlags(true, true, true);
      viewPort.setBackgroundColor(JMEDataTypeUtils.colorToColorRGBA(backgroundColor));
      viewPort.addProcessor(this);
      screenShotHelper = new JMEFastCaptureDevice(viewPort, jmeRenderer);
      stateManager.attach(screenShotHelper);

      if (addExtraVisuals)
      {
         setupShadows(viewPort);

         setUpPostProcesses();
      }

      this.viewportType = viewportType;

      if (viewportType.equals(ViewportType.MULTICAM) || viewportType.equals(ViewportType.CANVAS))
      {
         jmeRenderer.getContextManager().addJMEViewportAdapter(this);
         jmeInputManager = new JMEInputManager(jmeRenderer, rootNode, jmeCamera, flipY);
      }

      jmeRenderer.registerViewport(this);
   }

   @Override
   public void addFrameListener(Graphics3DFrameListener frameListener)
   {
      super.addFrameListener(frameListener);
      jmeRenderer.play();
   }

   public void setupViewport(double left, double right, double bottom, double top)
   {
      if (viewportType != ViewportType.MULTICAM)
      {
         throw new RuntimeException("Viewport is not setup for multicam rendering");
      }

      viewPort.setClearFlags(true, true, true);
      jmeCamera.setViewPort((float) left, (float) right, (float) bottom, (float) top);

   }

   @Override
   public Canvas getCanvas()
   {
      if (viewportType != ViewportType.CANVAS)
      {
         throw new RuntimeException("Viewport is not setup for canvas rendering");
      }

      /*
       * Only make panel when it is actually needed
       */
      if (panel == null)
      {
         if (context instanceof AwtPanelsContext)
         {
            AwtPanel awtPanel = ((AwtPanelsContext) context).createPanel(PaintMode.Accelerated);
            jmeRenderer.addRepaintListeners(awtPanel);
            awtPanel.attachTo(isMainViewport, viewPort);
            panel = awtPanel;
         }
         else if (context instanceof PBOAwtPanelsContext)
         {
            PBOAwtPanel awtPanel = ((PBOAwtPanelsContext) context).createPanel();
            jmeRenderer.addRepaintListeners(awtPanel);
            awtPanel.attachTo(isMainViewport, viewPort);
            panel = awtPanel;
         }
      }

      return panel;
   }

   @Override
   public void setupOffscreenView(int width, int height)
   {
      if (viewportType == ViewportType.CANVAS)
      {
         throw new RuntimeException("Viewport is setup for canvas rendering");
      }

      frameBuffer = new FrameBuffer(width, height, 1);
      viewPort.setOutputFrameBuffer(frameBuffer);
      viewPort.getCamera().resize(width, height, true);
      frameBuffer.setDepthBuffer(Format.Depth);
      frameBuffer.setColorBuffer(Format.RGBA8);
   }

   private boolean alreadyClosing = false;

   public void closeAndDispose()
   {
      if (alreadyClosing)
         return;
      alreadyClosing = true;

      viewPort.setEnabled(false);
      renderManager.removeMainView(viewPort);

      viewPort = null;
      renderManager = null;

      if (jmeCamera != null)
      {
         jmeCamera.closeAndDispose();
         jmeCamera = null;
      }
      jmeInputManager = null;

      context = null;

      panel = null;
      frameBuffer = null;
      viewportType = null;
      assetManager = null;

      if (contextSwitchedListeners != null)
      {
         contextSwitchedListeners.clear();
         contextSwitchedListeners = null;
      }

      if (screenShotHelper != null)
      {
         screenShotHelper.closeAndDispose();
         screenShotHelper = null;
      }
      stateManager = null;

      primaryLight = null;
      fpp = null;
      frameTimer = null;
   }

   public void closeViewportAdapter()
   {
      if (viewPort != null)
         viewPort.setEnabled(false);
      if (renderManager != null)
         renderManager.removeMainView(viewPort);
   }

   boolean addedPostProcessors = false;

   private void setupShadows(ViewPort viewPort)
   {
      DirectionalLightShadowRenderer dlsr = new DirectionalLightShadowRenderer(assetManager, 4096, 2);
      dlsr.setLight(primaryLight);
      dlsr.setLambda(0.8f);
      dlsr.setShadowIntensity(0.2f);
      dlsr.setEdgeFilteringMode(EdgeFilteringMode.PCF4);
      viewPort.addProcessor(dlsr);
   }

   public void setUpPostProcesses()
   {
      if (fpp == null)
      {
         //         fpp = new FilterPostProcessor(assetManager);
         //
         //         CartoonEdgeFilter f = new CartoonEdgeFilter();
         //
         //         f.setEdgeWidth(0.5f);
         //         f.setEdgeIntensity(0.5f);
         //
         ////
         //         fpp.addFilter(f);

         //       SSAOFilter ssaoFilter = new SSAOFilter(12.940201f, 43.928635f, 0.32999992f, 0.6059958f);
         //       fpp.addFilter(ssaoFilter);
         //         getViewPort().addProcessor(fpp);

      }

   }

   @Override
   public void setDefaultInputMappings()
   {
      jmeInputManager.registerWithInputManager();
   }

   @Override
   public void reset()
   {
      jmeInputManager.reset();
   }

   public void setClipDistances(double near, double far)
   {

   }

   public void setFieldOfView(double fieldOfView)
   {
   }

   @Override
   public CameraController getCameraController()
   {
      return jmeCamera.getController();
   }

   @Override
   public CaptureDevice getCaptureDevice()
   {
      return screenShotHelper;
   }

   public int getHeight()
   {
      return panel.getHeight();
   }

   public int getWidth()
   {
      return panel.getWidth();
   }

   public void setSize(int width, int height)
   {
      if (viewportType == ViewportType.CANVAS)
      {
         panel.setSize(width, height);
      }
      else
      {
         System.err.println("Resizing offscreen renderer, might crash!");
         setupOffscreenView(width, height);
      }
   }

   @Override
   public double getFieldOfView()
   {
      return 0;
   }

   private double convertToPhysicalDimension(int pixels)
   {
      double dPixels = pixels;
      double dpi = Toolkit.getDefaultToolkit().getScreenResolution();

      return dPixels / dpi * 0.0254;

   }

   @Override
   public double getPhysicalWidth()
   {
      return convertToPhysicalDimension(getWidth());
   }

   @Override
   public double getPhysicalHeight()
   {
      return convertToPhysicalDimension(getHeight());
   }

   public void updateCamera()
   {
      jmeCamera.updateCamera();
   }

   @Override
   public void setCameraController(CameraController cameraController)
   {
      jmeCamera.setCameraController(cameraController);
   }

   @Override
   public JMECamera getCamera()
   {
      return jmeCamera;
   }

   public boolean isMainViewport()
   {
      return isMainViewport;
   }

   /**
    * Calculate real world coordinates from x and y position on the screen
    *
    * @param x X coordinate on screen
    * @param y Y coordinate on screen
    * @param z Depth as relative value between near and far frustrum
    */
   @Override
   public Point3D getWorldCoordinatesFromScreenCoordinates(float x, float y, double z)
   {
      Vector3f worldCoordinates = jmeCamera.getWorldCoordinates(new Vector2f(x, y), (float) z);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(worldCoordinates);

      return JMEDataTypeUtils.jmeVector3fToJ3DPoint3d(worldCoordinates);
   }

   @Override
   public void addContextSwitchedListener(ContextSwitchedListener contextSwitchedListener)
   {
      contextSwitchedListeners.add(contextSwitchedListener);
   }

   public ViewPort getViewPort()
   {
      return viewPort;
   }

   public void attachScene(Spatial scene)
   {
      viewPort.attachScene(scene);
   }

   public void detachScene(Spatial scene)
   {
      viewPort.detachScene(scene);
   }

   @Override
   public void initialize(RenderManager rm, ViewPort vp)
   {

   }

   @Override
   public void reshape(ViewPort vp, int w, int h)
   {

   }

   @Override
   public boolean isInitialized()
   {
      return false;
   }

   @Override
   public void preFrame(float tpf)
   {

   }

   @Override
   public void postQueue(RenderQueue rq)
   {

   }

   @Override
   public void postFrame(FrameBuffer out)
   {
      if (alreadyClosing)
         return;

      double timePerFrame = frameTimer.lap();
      for (Graphics3DFrameListener listener : getFrameListeners())
      {
         listener.postFrame(timePerFrame);
      }
   }

   @Override
   public void cleanup()
   {

   }

   @Override
   public void setProfiler(AppProfiler profiler)
   {

   }
}
