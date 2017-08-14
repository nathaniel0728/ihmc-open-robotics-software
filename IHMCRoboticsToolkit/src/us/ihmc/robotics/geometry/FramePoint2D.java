package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>
 * Title: FramePoint2d
 * </p>
 *
 * <p>
 * Description: A FramePoint2d is a normal point associated with a specified reference frame
 * </p>
 *
 * @author Learning Locomotion Team
 */
public class FramePoint2D extends FrameTuple2D<FramePoint2D, Point2D> implements FramePoint2dReadOnly
{
   private static final long serialVersionUID = -1287148635726098768L;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();
   private FrameVector2D temporaryPointForYawing;

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double x, double y, String name)
   {
      super(referenceFrame, new Point2D(x, y), name);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(FrameTuple2D<?, ?> frameTuple2d)
   {
      this(frameTuple2d.referenceFrame, frameTuple2d.tuple.getX(), frameTuple2d.tuple.getY(), frameTuple2d.name);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 0.0, 0.0, null);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double x, double y)
   {
      this(referenceFrame, x, y, null);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double[] position)
   {
      this(referenceFrame, position[0], position[1], null);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, double[] position, String name)
   {
      this(referenceFrame, position[0], position[1], name);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, String name)
   {
      this(referenceFrame, 0.0, 0.0, name);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly position)
   {
      this(referenceFrame, position.getX(), position.getY(), null);
   }

   /**
    * FramePoint2d
    * <p/>
    * A normal point2d associated with a specific reference frame.
    */
   public FramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly position, String name)
   {
      this(referenceFrame, position.getX(), position.getY(), name);
   }

   public static FramePoint2D generateRandomFramePoint2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2D randomPoint = new FramePoint2D(zUpFrame, RandomNumbers.nextDouble(random, xMin, xMax),
                                                  RandomNumbers.nextDouble(random, yMin, yMax));

      return randomPoint;
   }

   public double distance(FramePoint2D framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      return this.tuple.distance(framePoint.tuple);
   }

   public double distanceSquared(FramePoint2D framePoint)
   {
      checkReferenceFrameMatch(framePoint);

      return this.tuple.distanceSquared(framePoint.tuple);
   }

   /**
    * Returns the point in this FramePoint2d.
    *
    * @return Point2d
    */
   public Point2D getPoint()
   {
      return tuple;
   }

   /**
    * Creates a new FramePoint based on the x and y components of this FramePoint2d
    */
   public FramePoint3D toFramePoint()
   {
      return new FramePoint3D(this.getReferenceFrame(), this.getX(), this.getY(), 0.0);
   }

   /**
    * Changes frame of this FramePoint2d to the given ReferenceFrame, projects into xy plane.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint2d into.
    */
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      applyTransform(temporaryTransformToDesiredFrame, false);
      this.referenceFrame = desiredFrame;
   }

   /**
    * Changes frame of this FramePoint2d to the given ReferenceFrame, projects into xy plane, and
    * returns a copy.
    *
    * @param desiredFrame ReferenceFrame to change the FramePoint2d into.
    * @return Copied FramePoint2d in the new reference frame.
    */
   public FramePoint2D changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FramePoint2D ret = new FramePoint2D(this);
      ret.changeFrameAndProjectToXYPlane(desiredFrame);
      return ret;
   }

   public void applyTransform(Transform transform, boolean requireTransformInXYPlane)
   {
      this.getGeometryObject().applyTransform(transform, requireTransformInXYPlane);
   }

   /**
    * yawAboutPoint
    *
    * @param pointToYawAbout FramePoint2d
    * @param yaw double
    * @return CartesianPositionFootstep
    */
   public void yawAboutPoint(FramePoint2D pointToYawAbout, FramePoint2D pointToPack, double yaw)
   {
      if (temporaryPointForYawing == null)
         temporaryPointForYawing = new FrameVector2D(this);
      else
         temporaryPointForYawing.setIncludingFrame(this);

      temporaryPointForYawing.sub(pointToYawAbout);

      temporaryTransformToDesiredFrame.setIdentity();
      temporaryTransformToDesiredFrame.setRotationYawAndZeroTranslation(yaw);

      temporaryPointForYawing.applyTransform(temporaryTransformToDesiredFrame);

      pointToPack.setIncludingFrame(pointToYawAbout);
      pointToPack.add(temporaryPointForYawing);
   }

}