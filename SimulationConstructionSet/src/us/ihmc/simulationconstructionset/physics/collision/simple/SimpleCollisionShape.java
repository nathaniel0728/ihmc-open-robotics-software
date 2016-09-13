
package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class SimpleCollisionShape implements CollisionShape
{
   private final CollisionShapeDescription collisionShapeDescription;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public SimpleCollisionShape(CollisionShapeDescription collisionShapeDescription)
   {
      this.collisionShapeDescription = collisionShapeDescription;
   }

   @Override
   public boolean isGround()
   {
      return false;
   }

   @Override
   public CollisionShapeDescription getDescription()
   {
      return collisionShapeDescription;
   }

   @Override
   public int getGroupMask()
   {
      return 0xFFFF;
   }

   @Override
   public int getCollisionMask()
   {
      return 0xFFFF;
   }

   @Override
   public void getTransformToWorld(RigidBodyTransform transformToWorldToPack)
   {
      transformToWorldToPack.set(transformToWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      this.transformToWorld.set(transformToWorld);
   }

}