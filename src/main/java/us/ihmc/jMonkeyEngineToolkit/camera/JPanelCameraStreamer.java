package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class JPanelCameraStreamer extends JPanel implements CameraStreamer
{
   private static final long serialVersionUID = -6832977971630763132L;
   private BufferedImage bufferedImage;

   public JPanelCameraStreamer()
   {
      super();
   }

   public synchronized void updateImage(BufferedImage bufferedImage)
   {
      this.bufferedImage = bufferedImage;
      repaint();
   }

   @Override
   public synchronized void updateImage(BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation,
                                        double fov)
   {
      updateImage(bufferedImage);
   }

   @Override
   protected synchronized void paintComponent(Graphics g)
   {
      if (bufferedImage != null)
      {
         g.drawImage(bufferedImage, 0, 0, this);
      }
   }

   public void createAndDisplayInNewWindow(String name)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", this);

      JFrame jFrame = new JFrame(name);
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);

      jFrame.pack();
      jFrame.setLocationByPlatform(true);
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }

   @Override
   public Point3D getCameraPosition()
   {
      return null;
   }

   @Override
   public Quaternion getCameraOrientation()
   {
      return null;
   }

   @Override
   public double getFieldOfView()
   {
      return 0;
   }

   @Override
   public boolean isReadyForNewData()
   {
      return true;
   }

   @Override
   public long getTimeStamp()
   {
      return 0;
   }

}