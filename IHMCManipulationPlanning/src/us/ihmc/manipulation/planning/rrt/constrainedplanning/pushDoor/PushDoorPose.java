package us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.manipulation.planning.trajectory.EndEffectorPose;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class PushDoorPose implements EndEffectorPose
{
   static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private PushDoor pushDoor;
   private double doorRotationAngle;
   private double pitchAngle;
   private RobotSide robotSide;
   
   public PushDoorPose(PushDoor pushDoor, RobotSide robotSide, double doorRotationAngle, double pitchAngle)
   {
      this.pushDoor = pushDoor;
      this.robotSide = robotSide;
      this.doorRotationAngle = doorRotationAngle;
      this.pitchAngle = pitchAngle;
   }
   
   public void setPitchAngle(double pitchAngle)
   {
      this.pitchAngle = pitchAngle;
   }

   @Override
   public Pose3D getEndEffectorPose()
   {  
      FramePose dummyFramePose = new FramePose(pushDoor.getDoorAxis());      
      dummyFramePose.changeFrame(worldFrame);
      
      Point3D translation = new Point3D(dummyFramePose.getPosition());
      RotationMatrix orientation = new RotationMatrix(dummyFramePose.getOrientation());
      
      RigidBodyTransform endEffectorRigidBody = new RigidBodyTransform(orientation, translation);
      
      endEffectorRigidBody.appendYawRotation(doorRotationAngle);
      
      endEffectorRigidBody.appendTranslation(0.0, pushDoor.getRadius(), pushDoor.getKnobHeight());
      
      if(robotSide == RobotSide.RIGHT)
      {
         endEffectorRigidBody.appendRollRotation(-Math.PI*0.5);
         endEffectorRigidBody.appendYawRotation(pitchAngle);
      }  
      else
      {
         endEffectorRigidBody.appendRollRotation(Math.PI*0.5);
         endEffectorRigidBody.appendYawRotation(-pitchAngle);
      }
         

      
      return new Pose3D(endEffectorRigidBody);
   }
}