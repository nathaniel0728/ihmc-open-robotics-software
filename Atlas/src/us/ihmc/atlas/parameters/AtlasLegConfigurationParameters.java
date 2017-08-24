package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;

public class AtlasLegConfigurationParameters extends LegConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public AtlasLegConfigurationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }
   
   @Override
   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return true; //FIXME
   }

   @Override
   /** {@inheritDoc} */
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(runningOnRealRobot ? 40.0 : 100.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   @Override
   /** {@inheritDoc} */
   public LegConfigurationGains getStraightLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(runningOnRealRobot ? 8.0 : 40.0);
      gains.setJointSpaceKd(runningOnRealRobot ? 3.0 : 6.0);

      gains.setUseActuatorSpacePositionControl(runningOnRealRobot);
      gains.setActuatorSpaceKp(30.0);

      return gains;
   }

   @Override
   /** {@inheritDoc} */
   public double getLegPrivilegedHighWeight()
   {
      return runningOnRealRobot ? 75.0 : 150.0;
   }

   @Override
   /** {@inheritDoc} */
   public double getFractionOfSwingToStraightenLeg()
   {
      return runningOnRealRobot ? 0.6 : 0.4;
   }

   @Override
   /** {@inheritDoc} */
   public double getKneeAngleWhenExtended()
   {
      return runningOnRealRobot ? 0.15 : 0.0;
   }

   @Override
   /** {@inheritDoc} */
   public double getKneeAngleWhenStraight()
   {
      return runningOnRealRobot ? 0.35 : 0.25;
   }
   
   public double getSpeedForSupportKneeStraightening()
   {
      return runningOnRealRobot ? 0.5 : 1.0;
   }
}
