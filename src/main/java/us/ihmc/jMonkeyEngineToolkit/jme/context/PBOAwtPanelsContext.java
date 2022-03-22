package us.ihmc.jMonkeyEngineToolkit.jme.context;

import java.util.ArrayList;
import java.util.List;

import com.jme3.input.JoyInput;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.TouchInput;
import com.jme3.input.awt.AwtKeyInput;
import com.jme3.input.awt.AwtMouseInput;
import com.jme3.opencl.Context;
import com.jme3.renderer.Renderer;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext;
import com.jme3.system.JmeSystem;
import com.jme3.system.SystemListener;
import com.jme3.system.Timer;

public class PBOAwtPanelsContext implements JmeContext
{
   private static final boolean DEBUG = false;

   private JmeContext actualContext;
   private AppSettings settings = new AppSettings(true);
   private SystemListener listener;
   private List<PBOAwtPanel> panels = new ArrayList<>();

   private AwtMouseInput mouseInput = new AwtMouseInput();
   private AwtKeyInput keyInput = new AwtKeyInput();

   private boolean lastThrottleState = false;

   private ArrayList<PBOAwtPanelListener> pboAwtPanelListeners = new ArrayList<>();

   private class AwtPanelsListener implements SystemListener
   {
      @Override
      public void initialize()
      {
         initInThread();
      }

      @Override
      public void reshape(int width, int height)
      {
         throw new IllegalStateException();
      }

      @Override
      public void update()
      {
         updateInThread();
      }

      @Override
      public void requestClose(boolean esc)
      {
         // shouldn't happen
         throw new IllegalStateException();
      }

      @Override
      public void gainFocus()
      {
         // shouldn't happen
         throw new IllegalStateException();
      }

      @Override
      public void loseFocus()
      {
         // shouldn't happen
         throw new IllegalStateException();
      }

      @Override
      public void handleError(String errorMsg, Throwable t)
      {
         listener.handleError(errorMsg, t);
      }

      @Override
      public void destroy()
      {
         destroyInThread();
      }
   }

   public void addPBOAwtPanelListener(PBOAwtPanelListener listener)
   {
      pboAwtPanelListeners.add(listener);
   }

   public void setInputSource(PBOAwtPanel panel)
   {
      if (!panels.contains(panel))
         throw new IllegalArgumentException();

      mouseInput.setInputSource(panel);
      keyInput.setInputSource(panel);
   }

   public List<PBOAwtPanel> getPanelList()
   {
      return panels;
   }

   @Override
   public Type getType()
   {
      return Type.OffscreenSurface;
   }

   @Override
   public void setSystemListener(SystemListener listener)
   {
      this.listener = listener;
   }

   @Override
   public AppSettings getSettings()
   {
      return settings;
   }

   @Override
   public Renderer getRenderer()
   {
      return actualContext.getRenderer();
   }

   @Override
   public MouseInput getMouseInput()
   {
      return mouseInput;
   }

   @Override
   public KeyInput getKeyInput()
   {
      return keyInput;
   }

   @Override
   public JoyInput getJoyInput()
   {
      return null;
   }

   @Override
   public TouchInput getTouchInput()
   {
      return null;
   }

   @Override
   public Timer getTimer()
   {
      return actualContext.getTimer();
   }

   @Override
   public boolean isCreated()
   {
      return actualContext != null && actualContext.isCreated();
   }

   @Override
   public boolean isRenderable()
   {
      return actualContext != null && actualContext.isRenderable();
   }

   public PBOAwtPanelsContext()
   {
   }

   public PBOAwtPanel createPanel()
   {
      if (alreadyDestroying)
         return null;

      PBOAwtPanel panel = new PBOAwtPanel(pboAwtPanelListeners);
      panels.add(panel);

      for (PBOAwtPanelListener listener : pboAwtPanelListeners)
      {
         listener.isCreated(panel);
      }

      return panel;
   }

   private void initInThread()
   {
      listener.initialize();
   }

   private void updateInThread()
   {
      if (alreadyDestroying)
         return;

      // Check if throttle required
      boolean needThrottle = true;

      for (PBOAwtPanel panel : panels)
      {
         if (panel.isActiveDrawing())
         {
            needThrottle = false;

            break;
         }
      }

      if (lastThrottleState != needThrottle)
      {
         lastThrottleState = needThrottle;

         if (lastThrottleState)
         {
            printIfDebug(getClass().getSimpleName() + ": Throttling update loop.");
         }
         else
         {
            printIfDebug(getClass().getSimpleName() + ": Ceased throttling update loop.");
         }
      }

      if (needThrottle)
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException ex)
         {
         }
      }

      if (!alreadyDestroying)
         listener.update();
   }

   boolean alreadyDestroying = false;

   private void destroyInThread()
   {
      if (alreadyDestroying)
         return;
      alreadyDestroying = true;

      listener.destroy();

      if (pboAwtPanelListeners != null)
      {
         pboAwtPanelListeners.clear();
         pboAwtPanelListeners = null;
      }

      if (panels != null)
      {
         for (PBOAwtPanel pboAwtPanel : panels)
         {
            pboAwtPanel.closeAndDispose();
         }

         panels.clear();
      }

      actualContext = null;
      settings = null;

      mouseInput = null;
      keyInput = null;
   }

   @Override
   public void setSettings(AppSettings settings)
   {
      this.settings.copyFrom(settings);
      this.settings.setRenderer(AppSettings.LWJGL_OPENGL32);

      if (actualContext != null)
      {
         actualContext.setSettings(settings);
      }
   }

   @Override
   public void create(boolean waitFor)
   {
      if (actualContext != null)
      {
         throw new IllegalStateException("Already created");
      }

      actualContext = JmeSystem.newContext(settings, Type.OffscreenSurface);
      actualContext.setSystemListener(new AwtPanelsListener());
      actualContext.create(waitFor);
   }

   @Override
   public void destroy(boolean waitFor)
   {
      if (actualContext == null)
         throw new IllegalStateException("Not created");

      // destroy parent context
      actualContext.destroy(waitFor);
   }

   @Override
   public void setTitle(String title)
   {
      // not relevant, ignore
   }

   @Override
   public void setAutoFlushFrames(boolean enabled)
   {
      // not relevant, ignore
   }

   @Override
   public void restart()
   {
      // only relevant if changing pixel format.
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   @Override
   public Context getOpenCLContext()
   {
      return null;
   }

}
