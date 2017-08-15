package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;

public class AtlasLeapOfFaithParameters extends LeapOfFaithParameters
{
   private final boolean runningOnRealRobot;
   
   public AtlasLeapOfFaithParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }
   
   /** {@inheritDoc} */
   @Override
   public double getFractionOfSwingToScaleFootWeight()
   {
      return runningOnRealRobot ? 0.96 : 0.92;
   }
   
   /** {@inheritDoc} */
   @Override
   public double getPelvisReachingFractionOfSwing()
   {
      return runningOnRealRobot ? 0.96 : 0.90;
   }

   /** {@inheritDoc} */
   @Override
   public double getPelvisReachingRollGain()
   {
      return runningOnRealRobot ? 0.5 : 1.0;
   }
   
   /** {@inheritDoc} */
   @Override
   public double getMinimumPelvisWeight()
   {
      return runningOnRealRobot ? 1.0 : 0.5;
   }
}
