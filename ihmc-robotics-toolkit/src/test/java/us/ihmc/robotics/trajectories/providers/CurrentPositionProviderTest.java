package us.ihmc.robotics.trajectories.providers;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CurrentPositionProviderTest
{
   private ReferenceFrame referenceFrame;
   private CurrentPositionProvider provider;

   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
   }

   @After
   public void tearDown()
   {
      referenceFrame = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      provider = new CurrentPositionProvider(null);
      provider = new CurrentPositionProvider(referenceFrame);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      provider = new CurrentPositionProvider(referenceFrame);
      FramePoint3D framePointToPack = new FramePoint3D(referenceFrame);

      provider.getPosition(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }
}
