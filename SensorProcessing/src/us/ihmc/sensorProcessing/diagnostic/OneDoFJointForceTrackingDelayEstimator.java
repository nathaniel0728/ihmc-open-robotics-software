package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelJointDataReadOnly;

public class OneDoFJointForceTrackingDelayEstimator implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final OneDoFJoint joint;
   private final LowLevelJointDataReadOnly output;
   private final DelayEstimatorBetweenTwoSignals delayEstimator;

   public OneDoFJointForceTrackingDelayEstimator(OneDoFJoint joint, LowLevelJointDataReadOnly outputDataToCheck, double dt, YoVariableRegistry parentRegistry)
   {
      this.joint = joint;
      this.output = outputDataToCheck;
      String jointName = joint.getName();
      registry = new YoVariableRegistry(jointName + "ForceTrackingDelayEstimator");
      delayEstimator = new DelayEstimatorBetweenTwoSignals(jointName + "ForceTracking", dt, registry);
   }

   @Override
   public void enable()
   {
      delayEstimator.enable();
   }

   @Override
   public void disable()
   {
      delayEstimator.disable();
   }

   @Override
   public void update()
   {
      delayEstimator.update(output.getDesiredTorque(), joint.getTauMeasured());
   }

   public void setAlphaFilterBreakFrequency(double delayEstimatorFilterBreakFrequency)
   {
      delayEstimator.setAlphaFilterBreakFrequency(delayEstimatorFilterBreakFrequency);
   }

   public void setEstimationParameters(double maxAbsoluteLead, double maxAbsoluteLag, double observationWindow)
   {
      delayEstimator.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
   }

   public boolean isEstimatingDelay()
   {
      return delayEstimator.isEstimatingDelay();
   }

   public double getEstimatedDelay()
   {
      return delayEstimator.getEstimatedDelay();
   }

   public double getCorrelation()
   {
      return delayEstimator.getCorrelationCoefficient();
   }
}
