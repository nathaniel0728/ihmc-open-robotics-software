package us.ihmc.commonWalkingControlModules.configurations;

import java.util.EnumMap;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class SmoothCMPPlannerParameters
{
   public enum CoPSupportPolygonNames
   {
      INITIAL_SWING_POLYGON, FINAL_SWING_POLYGON, SUPPORT_FOOT_POLYGON, INITIAL_DOUBLE_SUPPORT_POLYGON, FINAL_DOUBLE_SUPPORT_POLYGON, NULL
   };

   private final double modelScale;
   private final EnumMap<CoPPointName, Vector2D> copOffsetsInFootFrame = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, CoPSupportPolygonNames> copOffsetFrameNames = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> maxXCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> minXCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> maxYCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> minYCoPOffsets = new EnumMap<>(CoPPointName.class);
   
   private final EnumMap<CoPPointName, Boolean> constrainToMinMax = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Boolean> constrainToSupportPolygon = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, CoPSupportPolygonNames> stepLengthOffsetPolygon = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactor = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, Double> segmentTime = new EnumMap<>(CoPPointName.class);
   /**
    * Ordered list of CoP points to plan for each footstep
    */
   private final CoPPointName[] copPointsToPlan = {CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP, CoPPointName.BALL_COP};
   /**
    * Ordered list of transition time to a particular CoP point from the previous CoP point
    */
   private final double[] segmentDurations = {0.05, 0.05, 1.0}; // {from MIDFEET to HEEL, from HEEL to BALL, from BALL to MIDFEET}
   /**
    * Vector offsets relative to centroid of support polygon defined copOffsetFrames 
    */
   private final Vector2D[] copOffsets = {new Vector2D(0.0, 0.0), new Vector2D(0.0, -0.005), new Vector2D(0.0, 0.025)};
   private final CoPSupportPolygonNames[] copOffsetFrames = {CoPSupportPolygonNames.INITIAL_DOUBLE_SUPPORT_POLYGON, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON,
         CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON};

   /**
    * Order list of flags indicating whether specified bounding boxes should be used to constrain the CoP point
    */
   private final boolean[] constrainMinMaxFlags = {false, true, true};
   /**
    * Define the bounding box in sole frame
    */
   private final BoundingBox2D[] copOffsetLimits = {null, new BoundingBox2D(-0.04, -1, 0.03, 1), new BoundingBox2D(0.0, -1, 0.08, -1)};

   /**
    * Order list of flags indicating whether CoP should reside within the support polygon specified in copOffsetFrames
    */
   private final boolean[] constrainSupportPolygonFlags = {false, true, true};

   /**
    * Ordered list of fractions indicating whether CoP offset changes with step length
    */
   private final double[] stepLengthToCoPOffsetFactorValues = {0.0, 1.0 / 3.0, 1.0 / 3.0};
   /**
    * Defines the manner in which the step length is calculated
    */
   private final CoPSupportPolygonNames[] stepLengthOffsetPolygonValues = {CoPSupportPolygonNames.NULL, CoPSupportPolygonNames.INITIAL_SWING_POLYGON,
         CoPSupportPolygonNames.FINAL_SWING_POLYGON};

   /**
    * Final CoP name (chicken support will be used only for this point). In case 
    */
   private final CoPPointName endCoPName = CoPPointName.MIDFEET_COP;

   /**
    * Percentage chicken support
    */
   private final double chickenSuppportPercentage = 0.5;
   
   /**
    * Indicate the first CoP for the swing phase 
    */
   private final CoPPointName entryCoPName = CoPPointName.HEEL_COP;
   /**
    * Indicate the last CoP for the swing phase. Typically everything for this point should be determined from the final values otherwise computation is not possible
    */
   private final CoPPointName exitCoPName = CoPPointName.BALL_COP;
   
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
      return copPointsToPlan.length;
   }

   public EnumMap<CoPPointName, Double> getMaxCoPForwardOffsetsFootFrame()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         if(copOffsetLimits[i] != null)
            maxXCoPOffsets.put(copPointsToPlan[i], copOffsetLimits[i].getMaxX() * modelScale);
      }
      return maxXCoPOffsets;
   }

   public EnumMap<CoPPointName, Double> getMinCoPForwardOffsetsFootFrame()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         if(copOffsetLimits[i] != null)
            minXCoPOffsets.put(copPointsToPlan[i], copOffsetLimits[i].getMinX() * modelScale);
      }
      return minXCoPOffsets;
   }

   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsFootFrame()
   {
      Vector2D tempVec;
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         tempVec = copOffsets[i];
         tempVec.scale(modelScale);
         copOffsetsInFootFrame.put(copPointsToPlan[i], tempVec);
      }
      return copOffsetsInFootFrame;
   }
   
   public EnumMap<CoPPointName, CoPSupportPolygonNames> getSupportPolygonNames()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
      {
         copOffsetFrameNames.put(copPointsToPlan[i], copOffsetFrames[i]);
      }
      return copOffsetFrameNames;      
   }
   
   public CoPPointName getEntryCoPName()
   {
      return entryCoPName;
   }

   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   public CoPPointName getEndCoPName()
   {
      return endCoPName;
   }

   public CoPPointName[] getCoPPointsToPlan()
   {
      return copPointsToPlan;
   }

   @Deprecated
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   @Deprecated
   public double getSwingSplitFraction()
   {
      return 0.5;
   }

   @Deprecated
   public double getSwingDurationShiftFraction()
   {
      return 0.8;
   }

   @Deprecated
   public double getMaxDurationForSmoothingEntryToExitCoPSwitch()
   {
      return 0.5;
   }

   @Deprecated
   public double getStepLengthToCoPOffsetFactor()
   {
      return 1.0 / 3.0;
   }

   @Deprecated
   public double getMinTimeToSpendOnExitCoPInSingleSupport()
   {
      return 0.0;
   }

   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return modelScale * 0.01;
   }

   public EnumMap<CoPPointName, Boolean> getIsConstrainedToMinMaxFlags()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
         constrainToMinMax.put(copPointsToPlan[i], constrainMinMaxFlags[i]);
      return constrainToMinMax;
   }

   public EnumMap<CoPPointName, Boolean> getIsConstrainedToSupportPolygonFlags()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
         constrainToSupportPolygon.put(copPointsToPlan[i], constrainSupportPolygonFlags[i]);
      return constrainToSupportPolygon;
   }

   public EnumMap<CoPPointName, CoPSupportPolygonNames> getStepLengthToCoPOffsetFlags()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
         stepLengthOffsetPolygon.put(copPointsToPlan[i], stepLengthOffsetPolygonValues[i]);
      return stepLengthOffsetPolygon;
   }

   public EnumMap<CoPPointName, Double> getStepLengthToCoPOffsetFactors()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
         stepLengthToCoPOffsetFactor.put(copPointsToPlan[i], stepLengthToCoPOffsetFactorValues[i]);
      return stepLengthToCoPOffsetFactor;
   }

   public EnumMap<CoPPointName, Double> getSegmentTimes()
   {
      for (int i = 0; i < copPointsToPlan.length; i++)
         segmentTime.put(copPointsToPlan[i], segmentDurations[i]);
      return segmentTime;
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

   public double getPercentageChickenSupport()
   {
      return chickenSuppportPercentage;
   }
}