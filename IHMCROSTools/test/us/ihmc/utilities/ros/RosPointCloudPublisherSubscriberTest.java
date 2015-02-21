
package us.ihmc.utilities.ros;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.net.URISyntaxException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;

import org.junit.Test;

import sensor_msgs.PointCloud2;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.types.PointType;

@BambooPlan(planType=BambooPlanType.Flaky)
public class RosPointCloudPublisherSubscriberTest extends IHMCRosTestWithRosCore
{
   @AverageDuration
   @Test(timeout = 2000)
   public void testPubSubSinglePointXYZICloud() throws URISyntaxException, InterruptedException
   {
      testPubSubSingleCloud(PointType.XYZI);
   }

   @AverageDuration
   @Test(timeout = 2000)
   public void testPubSubSinglePointXYZRGBCloud() throws URISyntaxException, InterruptedException
   {
      testPubSubSingleCloud(PointType.XYZRGB);
   }

   private void testPubSubSingleCloud(PointType testPointType) throws URISyntaxException, InterruptedException
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "topicClientTestNode");

      // Test Data
      String testTopic = "/cloudTest";
      final String testFrameId = "/testFrame";
      final Point3d[] testPoints = new Point3d[] {new Point3d(1.0, 2.0, 3.0)};
      final float[] testIntensities = new float[] {1.0f};
      final Color3f[] testColor = new Color3f[] {new Color3f(1.0f, 2.0f, 3.0f)};


      // setup publisher
      final RosPointCloudPublisher publisher = new RosPointCloudPublisher(testPointType, true);
      rosMainNode.attachPublisher(testTopic, publisher);

      // setup subscriber
      final CountDownLatch latch = new CountDownLatch(1);
      RosPointCloudSubscriber subscriber = new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            super.unpackPointsAndIntensities(pointCloud);
            assertEquals(pointType, super.pointType);
            assertArrayEquals(testPoints, super.points);

            switch (super.pointType)
            {
               case XYZI :
                  assertArrayEquals(testIntensities, super.intensities, 1e-10f);

                  break;

               case XYZRGB :
                  assertArrayEquals(testColor, super.pointColors);
                  break;
            }

            assertEquals(testFrameId, pointCloud.getHeader().getFrameId());
            latch.countDown();
         }
      };
      rosMainNode.attachSubscriber(testTopic, subscriber);

      // go
      rosMainNode.execute();
      subscriber.wailTillRegistered();
      publisher.waitTillRegistered();

      switch (testPointType)
      {
         case XYZI :
            publisher.publish(testPoints, testIntensities, testFrameId);

            break;

         case XYZRGB :
            publisher.publish(testPoints, testColor, testFrameId);
      }

      assertTrue(latch.await(2, TimeUnit.SECONDS));
   }
}
