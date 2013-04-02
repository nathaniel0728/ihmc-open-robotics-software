package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements.LinearAccelerationMeasurementModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements.OrientationMeasurementModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.AngularAccelerationProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.CenterOfMassAccelerationProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.CenterOfMassPositionProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.CenterOfMassVelocityProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableOrientationAndCoMEstimatorCreator
{
   private static final int VECTOR3D_LENGTH = 3;

   private final RigidBody orientationEstimationLink;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final DenseMatrix64F angularAccelerationNoiseCovariance;

   private final List<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();
   private final List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();
   private final List<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

   public ComposableOrientationAndCoMEstimatorCreator(DenseMatrix64F angularAccelerationNoiseCovariance, RigidBody orientationEstimationLink,
         TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.angularAccelerationNoiseCovariance = angularAccelerationNoiseCovariance;
      this.orientationEstimationLink = orientationEstimationLink;
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
   }

   public void addOrientationSensorConfigurations(ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations)
   {
      for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
      {
         this.addOrientationSensorConfiguration(orientationSensorConfiguration);
      }
   }

   public void addOrientationSensorConfiguration(OrientationSensorConfiguration orientationSensorConfiguration)
   {
      orientationSensorConfigurations.add(orientationSensorConfiguration);
   }

   public void addAngularVelocitySensorConfigurations(ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations)
   {
      for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
      {
         addAngularVelocitySensorConfiguration(angularVelocitySensorConfiguration);
      }
   }

   public void addAngularVelocitySensorConfiguration(AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
   {
      this.angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
   }

   public void addLinearAccelerationSensorConfigurations(ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations)
   {
      for (LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration : linearAccelerationSensorConfigurations)
      {
         this.addLinearAccelerationSensorConfiguration(linearAccelerationSensorConfiguration);
      }
   }

   public void addLinearAccelerationSensorConfiguration(LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration)
   {
      this.linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
   }

   public OrientationEstimator createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, SixDoFJoint rootJoint, RigidBody estimationLink,
         ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort,
         ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort, YoVariableRegistry registry)
   {
      return new ComposableOrientationAndCoMEstimator("orientationEstimator", controlDT, rootJoint, estimationLink, estimationFrame, controlFlowGraph,
            desiredAngularAccelerationOutputPort, desiredCenterOfMassAccelerationOutputPort, registry);
   }

   private class ComposableOrientationAndCoMEstimator extends ComposableStateEstimator implements OrientationEstimator
   {
      private final ControlFlowOutputPort<FrameOrientation> orientationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityStatePort;
      private final ControlFlowOutputPort<FramePoint> centerOfMassPositionStatePort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityStatePort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularAccelerationStatePort;

      public ComposableOrientationAndCoMEstimator(String name, double controlDT, SixDoFJoint rootJoint, RigidBody estimationLink, ReferenceFrame estimationFrame,
            ControlFlowGraph controlFlowGraph, ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort,
            ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort, YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         orientationStatePort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         angularVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);
         angularAccelerationStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "AngularAcceleration", estimationFrame, registry);
         centerOfMassPositionStatePort = new YoFramePointControlFlowOutputPort(this, name + "CoMPosition", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMVelocity", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassAccelerationStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMAcceleration", ReferenceFrame.getWorldFrame(), registry);

         ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort = createProcessInputPort(VECTOR3D_LENGTH);
         controlFlowGraph.connectElements(desiredAngularAccelerationOutputPort, desiredAngularAccelerationInputPort);

         ControlFlowInputPort<FrameVector> desiredCenterOfMassAccelerationInputPort = createProcessInputPort(VECTOR3D_LENGTH);
         controlFlowGraph.connectElements(desiredCenterOfMassAccelerationOutputPort, desiredCenterOfMassAccelerationInputPort);

         addOrientationProcessModelElement();
         addAngularVelocityProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);
         addAngularAccelerationProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);

         addCoMPositionProcessModelElement();
         addCoMVelocityProcessModelElement(desiredCenterOfMassAccelerationInputPort);
         addCoMAccelerationProcessModelElement(desiredCenterOfMassAccelerationInputPort);

         for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
         {
            addOrientationSensor(estimationFrame, controlFlowGraph, orientationSensorConfiguration);
         }

         for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
         {
            addAngularVelocitySensor(estimationFrame, controlFlowGraph, angularVelocitySensorConfiguration);
         }

         for (LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration : linearAccelerationSensorConfigurations)
         {
            addLinearAccelerationSensor(estimationFrame, controlFlowGraph, linearAccelerationSensorConfiguration);
         }

         CenterOfMassBasedFullRobotModelUpdater centerOfMassBasedFullRobotModelUpdater = new CenterOfMassBasedFullRobotModelUpdater(twistCalculator, spatialAccelerationCalculator,
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort, orientationStatePort, angularVelocityStatePort,
               angularAccelerationStatePort, estimationLink, estimationFrame, rootJoint);

         addPostStateChangeRunnable(centerOfMassBasedFullRobotModelUpdater);

         initialize();
      }

      private void addAngularAccelerationProcessModelElement(ReferenceFrame estimationFrame, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
      {
         AngularAccelerationProcessModelElement processModelElement = new AngularAccelerationProcessModelElement("angularAcceleration", estimationFrame, registry,
               angularAccelerationStatePort, angularAccelerationInputPort);
         addProcessModelElement(angularAccelerationStatePort, processModelElement, VECTOR3D_LENGTH);
      }

      private void addCoMVelocityProcessModelElement(ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         CenterOfMassVelocityProcessModelElement centerOfMassVelocityProcessModelElement = new CenterOfMassVelocityProcessModelElement(
               centerOfMassVelocityStatePort, centerOfMassAccelerationInputPort, "CoMVelocity", registry);
         addProcessModelElement(centerOfMassVelocityStatePort, centerOfMassVelocityProcessModelElement, VECTOR3D_LENGTH);
      }

      private void addCoMPositionProcessModelElement()
      {
         CenterOfMassPositionProcessModelElement centerOfMassPositionProcessModelElement = new CenterOfMassPositionProcessModelElement(
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, "CoMPosition", registry);
         addProcessModelElement(centerOfMassPositionStatePort, centerOfMassPositionProcessModelElement, VECTOR3D_LENGTH);
      }

      private void addCoMAccelerationProcessModelElement(ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         CenterOfMassAccelerationProcessModelElement centerOfMassAccelerationProcessModelElement = new CenterOfMassAccelerationProcessModelElement(
               "CoMAcceleration", registry, centerOfMassAccelerationStatePort, centerOfMassAccelerationInputPort);
         addProcessModelElement(centerOfMassAccelerationStatePort, centerOfMassAccelerationProcessModelElement, VECTOR3D_LENGTH);
      }

      private void addOrientationProcessModelElement()
      {
         ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityStatePort, orientationStatePort, "orientation",
               registry);
         addProcessModelElement(orientationStatePort, orientationProcessModelElement, VECTOR3D_LENGTH);
      }

      private void addAngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
      {
         AngularVelocityProcessModelElement angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame,
               angularVelocityStatePort, angularAccelerationInputPort, "angularVelocity", registry);

         angularVelocityProcessModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityStatePort, angularVelocityProcessModelElement, VECTOR3D_LENGTH);
      }

      private void addOrientationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            OrientationSensorConfiguration orientationSensorConfiguration)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<Matrix3d> orientationMeasurementPort = createMeasurementInputPort(VECTOR3D_LENGTH);
         String name = orientationSensorConfiguration.getName();
         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationStatePort,
               orientationMeasurementPort, estimationFrame, measurementFrame, name, registry);
         orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);

         addMeasurementModelElement(orientationMeasurementPort, orientationMeasurementModel);
         controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementPort);
      }

      private void addAngularVelocitySensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
      {
         String biasName = angularVelocitySensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
         ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = createMeasurementInputPort(VECTOR3D_LENGTH);

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);

         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement, VECTOR3D_LENGTH);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityStatePort,
               biasPort, angularVelocityMeasurementPort, orientationEstimationLink, estimationFrame, measurementLink, measurementFrame, twistCalculator, name,
               registry);
         angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);

         addMeasurementModelElement(angularVelocityMeasurementPort, angularVelocityMeasurementModel);
         controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
      }

      private void addLinearAccelerationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration)
      {
         String biasName = linearAccelerationSensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = linearAccelerationSensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = linearAccelerationSensorConfiguration.getLinearAccelerationMeasurementLink();
         ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort = createMeasurementInputPort(VECTOR3D_LENGTH);

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);

         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = linearAccelerationSensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement, VECTOR3D_LENGTH);
         String name = linearAccelerationSensorConfiguration.getName();

         DenseMatrix64F linearAccelerationNoiseCovariance = linearAccelerationSensorConfiguration.getLinearAccelerationNoiseCovariance();

         // TODO: Get the measurement model working.

         double gZ = linearAccelerationSensorConfiguration.getGravityZ();

         LinearAccelerationMeasurementModelElement linearAccelerationMeasurementModel = new LinearAccelerationMeasurementModelElement(name, registry,
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort, orientationStatePort, angularVelocityStatePort,
               angularAccelerationStatePort, biasPort, linearAccelerationMeasurementInputPort, twistCalculator, spatialAccelerationCalculator, measurementLink,
               measurementFrame, orientationEstimationLink, estimationFrame, gZ);

         linearAccelerationMeasurementModel.setNoiseCovariance(linearAccelerationNoiseCovariance);

         addMeasurementModelElement(linearAccelerationMeasurementInputPort, linearAccelerationMeasurementModel);
         controlFlowGraph.connectElements(linearAccelerationSensorConfiguration.getOutputPort(), linearAccelerationMeasurementInputPort);
      }
      
      public FrameOrientation getEstimatedOrientation()
      {
         return orientationStatePort.getData();
      }

      public FrameVector getEstimatedAngularVelocity()
      {
         return angularVelocityStatePort.getData();
      }

      public void setEstimatedOrientation(FrameOrientation orientation)
      {
         orientationStatePort.setData(orientation);
      }

      public void setEstimatedAngularVelocity(FrameVector angularVelocity)
      {
         angularVelocityStatePort.setData(angularVelocity);
      }

      public DenseMatrix64F getCovariance()
      {
         return kalmanFilter.getCovariance();
      }

      public DenseMatrix64F getState()
      {
         return kalmanFilter.getState();
      }

      public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
      {
         kalmanFilter.setState(x, covariance);
      }
   }
}
