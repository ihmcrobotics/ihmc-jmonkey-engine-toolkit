package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import static us.ihmc.robotics.Assert.assertTrue;

import java.time.Duration;
import java.util.concurrent.LinkedBlockingQueue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.jMonkeyEngineToolkit.jme.lidar.manual.JMELidar120FovTest;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.manual.JMELidar360FovTest;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.manual.JMELidar60FovTest;
import us.ihmc.jMonkeyEngineToolkit.jme.lidar.manual.JMELidarSphere270FovTest;

/**
 * For some reason, this class cannot be run with -ea (enable assertions) An internal JME assert
 * fails if you do. Not sure what to do about it, but it'll likely be a difficult problem to solve.
 */
@Tag("jme")
public class JMEGPULidarTest implements LidarTestListener
{
   private static final boolean TEST_MANUALLY = false;
   private JMEGPULidarTestEnviroment lidarTest;
   private LidarTestParameters parameters;
   private boolean stop = false;
   private final LinkedBlockingQueue<ScanPair> scanPairs = new LinkedBlockingQueue<>();
   private double averageDifference = 0.0;
   private long numScans = 0;

   @AfterEach
   public void tearDown()
   {
      System.out.println("Average difference: " + averageDifference / numScans + " Number of Scans: " + numScans);

      assertTrue("Number of scans incorrect: " + numScans, numScans > 1000);

      numScans = 0;
      averageDifference = 0.0;

      lidarTest.getWorld().stop();
   }

   @Test // timeout = 30000
   public void test60DegreeFieldOfView()
   {
      parameters = new JMELidar60FovTest();
      doATest(parameters);
   }

   @Test // timeout = 30000
   public void test120DegreeFieldOfView()
   {
      parameters = new JMELidar120FovTest();
      doATest(parameters);
   }

   @Disabled
   @Test // timeout = 30000
   public void test360DegreeFieldOfView()
   {
      parameters = new JMELidar360FovTest();
      doATest(parameters);
   }

   @Disabled
   @Test // timeout = 30000
   public void test270DegreeFieldOfView()
   {
      parameters = new JMELidarSphere270FovTest();
      doATest(parameters);
   }

   private void doATest(LidarTestParameters parameters)
   {
      Assertions.assertTimeout(Duration.ofSeconds(30), () ->
      {
         lidarTest = new JMEGPULidarTestEnviroment();

         if (TEST_MANUALLY)
            lidarTest.testManually(parameters, this);
         else
            lidarTest.testAutomatically(parameters, this);

         beginAssertingLidarScans();
      });
   }

   private void beginAssertingLidarScans()
   {
      while (!stop)
      {
         if (!scanPairs.isEmpty())
         {
            ScanPair pair = scanPairs.poll();
            assertTrue(pair.gpuScan.epsilonEquals(pair.traceScan, 1.0e-7, (float) parameters.getGpuVsTraceTolerance()));

            recordStatistics(pair.gpuScan, pair.traceScan);
         }
      }
   }

   private void recordStatistics(LidarTestScan gpuScan, LidarTestScan traceScan)
   {
      for (int i = 0; i < parameters.getScansPerSweep(); i++)
      {
         averageDifference += traceScan.getRange(i) - gpuScan.getRange(i);
         numScans++;
      }
   }

   @Override
   public void notify(LidarTestScan gpuScan, LidarTestScan traceScan)
   {
      scanPairs.add(new ScanPair(gpuScan, traceScan));
   }

   @Override
   public void stop()
   {
      stop = true;
   }

   private class ScanPair
   {
      private final LidarTestScan gpuScan;
      private final LidarTestScan traceScan;

      public ScanPair(LidarTestScan gpuScan, LidarTestScan traceScan)
      {
         this.gpuScan = gpuScan;
         this.traceScan = traceScan;
      }
   }
}
