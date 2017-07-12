package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TransferCoPTrajectory extends CoPTrajectory
{
   private final static CoPTrajectoryType type = CoPTrajectoryType.TRANSFER;   
   public TransferCoPTrajectory(String namePrefix, int stepNumber, int maxNumberOfSegments, YoVariableRegistry registry)
   {
      super(namePrefix, stepNumber, maxNumberOfSegments, type, registry);
   }  
}
