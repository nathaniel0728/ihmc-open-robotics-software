package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class SmoothCMPPlannerParameters
{
   private final double modelScale;
   private final EnumMap<CoPPointName, Vector2D> copOffsetsFootFrame = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> maxXCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> minXCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> maxYCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> minYCoPOffsets = new EnumMap<>(CoPPointName.class);

   /**
    * List of CoP points to plan in the order of planning
    */
   private final List<CoPPointName> copPointsToPlan = Arrays.asList(CoPPointName.HEEL_COP, CoPPointName.BALL_COP);
   /**
    * CoP offsets in foot frame
    */
   private final Vector2D[] copOffsets = {new Vector2D(0.0, -0.005), new Vector2D(0.0, 0.025)};
   private final BoundingBox2D[] copOffsetLimits = {new BoundingBox2D(-0.04, -1, 0.03, 1), new BoundingBox2D(0.0, -1, 0.08, -1)};
   /**
    * Final CoP name (chicken support will be used only for this point)
    */
   private final CoPPointName endCoP = CoPPointName.MIDFEET_COP;
   /**
    * Percentage chicken support
    */
   private final double chickenSuppportPercentage = 0.5;

   public SmoothCMPPlannerParameters()
   {
      this(1.0);
   }

   public SmoothCMPPlannerParameters(double modelScale)
   {
      this.modelScale = modelScale;
   }

   /**
    * How many footsteps the ICP planner will use to build the plan. The more the better, but will
    * increase the number of YoVariables and increase the computation time. The values 3 and 4 seem
    * to be good.
    */
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   public int getNumberOfWayPointsPerFoot()
   {
      return copPointsToPlan.size();
   }

   public EnumMap<CoPPointName, Double> getMaxCoPForwardOffsetsFootFrame()
   {
      for (int i = 0; i < copPointsToPlan.size(); i++)
      {
         maxXCoPOffsets.put(copPointsToPlan.get(i), copOffsetLimits[i].getMaxX() * modelScale);
      }
      return maxXCoPOffsets;
   }

   public EnumMap<CoPPointName, Double> getMinCoPForwardOffsetsFootFrame()
   {
      for (int i = 0; i < copPointsToPlan.size(); i++)
      {
         minXCoPOffsets.put(copPointsToPlan.get(i), copOffsetLimits[i].getMinX() * modelScale);
      }
      return minXCoPOffsets;
   }

   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsFootFrame()
   {
      Vector2D tempVec;
      for (int i = 0; i < copPointsToPlan.size(); i++)
      {
         tempVec = copOffsets[i];
         tempVec.scale(modelScale);
         copOffsetsFootFrame.put(copPointsToPlan.get(i), tempVec);
      }
      return copOffsetsFootFrame;
   }
   
   public CoPPointName getEntryCoPName()
   {
      return copPointsToPlan.get(0);
   }
   
   public CoPPointName getExitCoPName()
   {
      return copPointsToPlan.get(copPointsToPlan.size() -1);
   }

   public List<CoPPointName> getCoPPointsToPlan()
   {
      return copPointsToPlan;
   }
   
   // Need to understand what all of these do 
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   public double getSwingSplitFraction()
   {
      return 0.5;
   }

   public double getSwingDurationShiftFraction()
   {
      return 0.8;
   }

   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   public double getMaxDurationForSmoothingEntryToExitCoPSwitch()
   {
      return 0.5;
   }

   public double getStepLengthToCoPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   /**
    * Provides a list of the alphas that denote the percentage of time taken to transition from one CoP way point to another.
    * Summation of the list must be equal to 1. 
    * @return
    */
   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.LINEAR;
   }
}
