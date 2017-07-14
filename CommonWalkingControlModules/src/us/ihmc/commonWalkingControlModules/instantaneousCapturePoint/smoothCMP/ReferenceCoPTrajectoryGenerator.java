package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import javax.management.RuntimeErrorException;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CoPPolynomialTrajectoryPlannerInterface;
import us.ihmc.commons.PrintTools;
import us.ihmc.convexOptimization.qpOASES.returnValue;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

// TODO 1) Modify initializeParamters() to have only things that should be adjusted on the fly there

public class ReferenceCoPTrajectoryGenerator implements CoPPolynomialTrajectoryPlannerInterface, ReferenceCoPTrajectoryGeneratorInterface
{
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double COP_POINT_SIZE = 0.005;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix;

   // Waypoint planning parameters
   private final SideDependentList<EnumMap<CoPPointName, YoFrameVector2d>> copOffsets = new SideDependentList<>();
   private final EnumMap<CoPPointName, CoPSupportPolygonNames> copSupportPolygon;
   private final EnumMap<CoPPointName, Boolean> isConstrainedToSupportPolygonFlags;

   private final EnumMap<CoPPointName, Boolean> isConstrainedToMinMaxFlags;
   private final EnumMap<CoPPointName, YoDouble> maxCoPOffsets = new EnumMap<CoPPointName, YoDouble>(CoPPointName.class);
   private final EnumMap<CoPPointName, YoDouble> minCoPOffsets = new EnumMap<CoPPointName, YoDouble>(CoPPointName.class);

   private final EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactors;
   private final EnumMap<CoPPointName, CoPSupportPolygonNames> stepLengthOffsetReferencePolygons;

   private final YoDouble safeDistanceFromCoPToSupportEdges;
   private final YoDouble percentageChickenSupport;
   private final CoPPointName entryCoPName;
   private final CoPPointName exitCoPName;
   private final CoPPointName endCoPName;
   private final CoPPointName[] copPointList;

   // Trajectory planning parameters
   private final EnumMap<CoPPointName, Double> segmentTimes;

   // State variables 
   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberOfPointsPerFoot;
   private final YoInteger numberOfFootstepsToConsider;
   private final YoEnum<CoPSplineType> orderOfSplineInterpolation;

   // Output variables
   private final YoBoolean isDoneWalking;
   private final List<CoPPointsInFoot> copLocationWaypoints = new ArrayList<>();
   private final List<TransferCoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private final List<SwingCoPTrajectory> swingCoPTrajectories = new ArrayList<>();

   // Runtime variables
   private FramePoint desiredCoPPosition = new FramePoint();
   private FrameVector desiredCoPVelocity = new FrameVector();
   private FrameVector desiredCoPAcceleration = new FrameVector();
   private int numberOfFootstepsConsidered; // Always sync this with the YoInteger numberOfFootstepsToConsider
   private int numberOfCoPPointsPerFoot; // Always sync this with the YoInteger numberOfPointsPerFoot

   private int footstepIndex = 0;
   private int plannedFootstepIndex = -1;
   private int numberOfSwingSegments;
   private int numberOfTransferSegments;
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FrameConvexPolygon2d tempDoubleSupportPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSupportFootPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSwingFootInitialPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSwingFootFinalPolygon = new FrameConvexPolygon2d();

   // Temp variables for computation only
   private double tempDouble;
   private FootstepData currentFootstepData;
   private FrameConvexPolygon2d framePolygonReference;
   private FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private ConvexPolygonShrinker polygonShrinker = new ConvexPolygonShrinker();
   private FramePoint tempFramePoint = new FramePoint();
   private FramePoint2d tempFramePoint2d = new FramePoint2d();

   // Planner level overrides (the planner knows better!)
   private static final int maxNumberOfPointsInFoot = 4;
   private static final int maxNumberOfFootstepsToConsider = 4;

   // Input data
   private final RecyclingArrayList<FootstepData> upcomingFootstepsData = new RecyclingArrayList<FootstepData>(maxNumberOfFootstepsToConsider,
                                                                                                               FootstepData.class);
   private final YoInteger numberOfUpcomingFootsteps;

   public ReferenceCoPTrajectoryGenerator(String namePrefix, SmoothCMPPlannerParameters plannerParameters, BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberOfFootstepsToConsider,
                                          List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> swingSplitFractions,
                                          List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.segmentTimes = plannerParameters.getSegmentTimes();

      this.copSupportPolygon = plannerParameters.getSupportPolygonNames();
      this.isConstrainedToSupportPolygonFlags = plannerParameters.getIsConstrainedToSupportPolygonFlags();
      this.isConstrainedToMinMaxFlags = plannerParameters.getIsConstrainedToMinMaxFlags();
      this.stepLengthToCoPOffsetFactors = plannerParameters.getStepLengthToCoPOffsetFactors();
      this.stepLengthOffsetReferencePolygons = plannerParameters.getStepLengthToCoPOffsetFlags();

      EnumMap<CoPPointName, Double> maxCoPOffsets = plannerParameters.getMaxCoPForwardOffsetsFootFrame();
      EnumMap<CoPPointName, Double> minCoPOffsets = plannerParameters.getMinCoPForwardOffsetsFootFrame();
      this.copPointList = plannerParameters.getCoPPointsToPlan();
      for (int i = 0; i < copPointList.length; i++)
      {
         if (isConstrainedToMinMaxFlags.get(copPointList[i]))
         {
            YoDouble maxCoPOffset = new YoDouble("maxCoPForwardOffset" + copPointList[i].toString(), registry);
            YoDouble minCoPOffset = new YoDouble("minCoPForwardOffset" + copPointList[i].toString(), registry);
            maxCoPOffset.set(maxCoPOffsets.get(copPointList[i]));
            minCoPOffset.set(minCoPOffsets.get(copPointList[i]));
            this.maxCoPOffsets.put(copPointList[i], maxCoPOffset);
            this.minCoPOffsets.put(copPointList[i], minCoPOffset);
         }
      }
      safeDistanceFromCoPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdges", registry);

      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
      percentageChickenSupport.set(plannerParameters.getPercentageChickenSupport());

      this.entryCoPName = plannerParameters.getEntryCoPName();
      this.exitCoPName = plannerParameters.getExitCoPName();
      this.endCoPName = plannerParameters.getEndCoPName();
      processCoPList();

      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      for (RobotSide robotSide : RobotSide.values)
      {
         supportFootPolygonsInSoleZUpFrames.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(robotSide));
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         EnumMap<CoPPointName, YoFrameVector2d> copUserOffsets = new EnumMap<>(CoPPointName.class);

         for (int i = 0; i < copPointList.length; i++)
         {
            YoFrameVector2d copUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "CoPConstantOffset" + copPointList[i].toString(), null, registry);
            copUserOffsets.put(copPointList[i], copUserOffset);
         }
         this.copOffsets.put(robotSide, copUserOffsets);
      }

      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      setNumberOfPointsPerFoot(plannerParameters.getNumberOfWayPointsPerFoot());

      this.numberOfFootstepsToConsider = new YoInteger(namePrefix + "NumberOfFootstepsToConsider", registry);
      setNumberOfFootstepsToConsider(plannerParameters.getNumberOfFootstepsToConsider());

      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.orderOfSplineInterpolation.set(plannerParameters.getOrderOfCoPInterpolation());

      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};
      // Need two additional CoPPoints in Foot to store the initial and final footstep CoPs
      copLocationWaypoints.add(new CoPPointsInFoot(0, framesToRegister, parentRegistry));
      for (int i = 1; i <= this.numberOfFootstepsConsidered; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(i, framesToRegister, registry));
      }
      copLocationWaypoints.add(new CoPPointsInFoot(this.numberOfFootstepsConsidered + 1, framesToRegister, parentRegistry));

      for (int i = 0; i < this.numberOfFootstepsConsidered; i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, i, orderOfSplineInterpolation.getEnumValue(),
                                                                                 this.numberOfTransferSegments, registry);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(namePrefix, i, orderOfSplineInterpolation.getEnumValue(), this.numberOfSwingSegments,
                                                                        registry);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, this.numberOfFootstepsConsidered,
                                                                              orderOfSplineInterpolation.getEnumValue(), this.numberOfTransferSegments,
                                                                              registry);
      transferCoPTrajectories.add(transferCoPTrajectory);

      this.numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);
      parentRegistry.addChild(registry);
      initializeParameters(plannerParameters);
      clear();
   }

   private void processCoPList()
   {
      int entryCoPIndex = getCoPIndex(entryCoPName);
      int exitCoPIndex = getCoPIndex(exitCoPName);
      this.numberOfSwingSegments = entryCoPIndex < exitCoPIndex ? exitCoPIndex - entryCoPIndex : copPointList.length + exitCoPIndex - entryCoPIndex;
      this.numberOfTransferSegments = copPointList.length - this.numberOfSwingSegments;
   }

   private int getCoPIndex(CoPPointName copPointToSearch)
   {
      for (int index = 0; index < copPointList.length; index++)
         if (copPointList[index] == copPointToSearch)
            return index;
      return -1;
   }

   private void setNumberOfPointsPerFoot(int numberOfPointsPerFoot)
   {
      this.numberOfPointsPerFoot.set(Math.min(numberOfPointsPerFoot, maxNumberOfPointsInFoot));
      this.numberOfCoPPointsPerFoot = this.numberOfPointsPerFoot.getIntegerValue();
   }

   private void setNumberOfFootstepsToConsider(int numberOfFootstepsToConsider)
   {
      this.numberOfFootstepsToConsider.set(Math.min(numberOfFootstepsToConsider, maxNumberOfFootstepsToConsider));
      this.numberOfFootstepsConsidered = this.numberOfFootstepsToConsider.getIntegerValue();
   }

   @Override
   public void initializeParameters(SmoothCMPPlannerParameters parameters)
   {
      safeDistanceFromCoPToSupportEdges.set(parameters.getCoPSafeDistanceAwayFromSupportEdges());

      EnumMap<CoPPointName, Vector2D> copOffsets = parameters.getCoPOffsetsFootFrame();
      CoPPointName[] copPointList = parameters.getCoPPointsToPlan();
      for (int i = 0; i < copPointList.length; i++)
      {
         setSymmetricCoPConstantOffsets(copPointList[i], copOffsets.get(copPointList[i]));
      }
   }

   @Override
   public void setSymmetricCoPConstantOffsets(CoPPointName copPointName, Vector2D heelOffset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d copUserOffset = copOffsets.get(robotSide).get(copPointName);
         copUserOffset.setX(heelOffset.getX());
         copUserOffset.setY(robotSide.negateIfLeftSide(heelOffset.getY()));
      }
   }

   @Override
   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int footIndex = 0; footIndex < copLocationWaypoints.size(); footIndex++)
      {
         CoPPointsInFoot copPointsInFoot = copLocationWaypoints.get(footIndex);
         List<CoPPointName> copPointNames = copPointsInFoot.getCoPPointList();
         for (int i = 0; i < copPointNames.size(); i++)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(footIndex + "Foot CoP Waypoint" + copPointNames.get(i).toString(),
                                                             copPointsInFoot.getWaypointInWorldFrameReadOnly(copPointNames.get(i)), COP_POINT_SIZE,
                                                             YoAppearance.Green(), GraphicType.BALL);
            yoGraphicsList.add(copViz);
            artifactList.add(copViz.createArtifact());
         }
      }
   }

   @Override
   public void updateListeners()
   {
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).notifyVariableChangedListeners();
   }

   @Override
   public void clear()
   {
      upcomingFootstepsData.clear();
      numberOfUpcomingFootsteps.set(0);
      desiredCoPPosition.setToNaN();
      desiredCoPVelocity.setToNaN();
      desiredCoPAcceleration.setToNaN();
      clearPlan();
   }

   public void clearPlan()
   {
      footstepIndex = 0;
      plannedFootstepIndex = -1;
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).reset();
      for (int i = 0; i < transferCoPTrajectories.size(); i++)
         transferCoPTrajectories.get(i).reset();
      for (int i = 0; i < swingCoPTrajectories.size(); i++)
         swingCoPTrajectories.get(i).reset();
   }

   @Override
   public int getNumberOfFootstepsRegistered()
   {
      return numberOfUpcomingFootsteps.getIntegerValue();
   }

   @Override
   public void initializeForTransfer(double currentTime)
   {
      this.initialTime = currentTime;
      this.activeTrajectory = transferCoPTrajectories.get(0);
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      this.initialTime = currentTime;
      this.activeTrajectory = swingCoPTrajectories.get(0);
   }

   @Override
   public void update(double currentTime)
   {
      if (activeTrajectory != null)
         activeTrajectory.update(currentTime - this.initialTime, desiredCoPPosition, desiredCoPVelocity, desiredCoPAcceleration);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint desiredCoPToPack)
   {
      desiredCoPToPack.setIncludingFrame(desiredCoPPosition);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack)
   {
      getDesiredCenterOfPressure(desiredCoPToPack);
      desiredCoPVelocityToPack.setIncludingFrame(desiredCoPVelocity);
   }

   @Override
   public void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack)
   {
      desiredCoPToPack.set(desiredCoPPosition);
   }

   @Override
   public void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack, YoFrameVector desiredCoPVelocityToPack)
   {
      getDesiredCenterOfPressure(desiredCoPToPack);
      desiredCoPVelocityToPack.set(desiredCoPVelocity);
   }

   @Override
   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      footstepIndex = 0;
      updateFootPolygons();
      // Put first CoP as per chicken support computations in case starting from rest
      if (atAStop || numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         computeMidFeetPointWithChickenSupportForInitialTransfer(tempFramePoint);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(endCoPName, 0.0, tempFramePoint);
      }
      // Put first CoP at the exitCoP of the swing foot if not starting from rest 
      else
      {
         computeInitialExitCoP(tempFramePoint, upcomingFootstepsData.get(footstepIndex).getSwingSide());
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempFramePoint);
      }
      computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
      generateCoPTrajectoriesFromWayPoints(CoPTrajectoryType.TRANSFER);
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      footstepIndex = 0;
      updateFootPolygons();
      if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
         return;
      else
      {
         if (copPointList[0] == entryCoPName)
         {
            computeInitialExitCoP(tempFramePoint, upcomingFootstepsData.get(footstepIndex).getSwingSide());
            copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempFramePoint);
            computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
         }
         else
            computeCoPPointsForUpcomingFootsteps(footstepIndex);
      }
      generateCoPTrajectoriesFromWayPoints(CoPTrajectoryType.SWING);
   }

   private void computeMidFeetPointWithChickenSupportForInitialTransfer(FramePoint framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, currentSupportFootPolygon, currentSwingFootInitialPolygon);
   }

   private void computeMidFeetPointWithChickenSupportForFinalTransfer(FramePoint framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, currentSupportFootPolygon, currentSwingFootFinalPolygon);
   }

   /**
    * Assumes foot polygon and double support polygon has been updated
    * @param framePointToPack
    * @param transferToSide
    */
   private void computeMidFeetPointWithChickenSupport(FramePoint framePointToPack, FrameConvexPolygon2d supportFootPolygon,
                                                      FrameConvexPolygon2d swingFootPolygon)
   {
      this.tempDouble = MathTools.clamp(percentageChickenSupport.getDoubleValue(), 0.0, 1.0);
      if (this.tempDouble < 0.5)
      {
         this.tempDouble *= 2.0;
         this.framePolygonReference = supportFootPolygon;
      }
      else
      {
         this.tempDouble = (0.5 - this.tempDouble) * 2.0;
         this.framePolygonReference = swingFootPolygon;
      }
      this.tempDoubleSupportPolygon.changeFrame(worldFrame);
      this.framePolygonReference.changeFrame(worldFrame);
      this.tempFramePoint2d.interpolate(this.tempDoubleSupportPolygon.getCentroid(), this.framePolygonReference.getCentroid(), this.tempDouble);
      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
   }

   /**
    * Calculates the exit CoP for a given footstep
    * @param framePointToPack
    * @param robotSide
    */
   private void computeInitialExitCoP(FramePoint framePointToPack, RobotSide swingSide)
   {
      setInitialCoPPointToPolygonOrigin(tempFramePoint2d, exitCoPName);
      this.tempDouble = copOffsets.get(swingSide).get(exitCoPName).getX() + getInitialStepLengthToCoPOffset(exitCoPName);

      if (isConstrainedToMinMaxFlags.get(exitCoPName))
         this.tempDouble = MathTools.clamp(this.tempDouble, minCoPOffsets.get(exitCoPName).getDoubleValue(), maxCoPOffsets.get(exitCoPName).getDoubleValue());
      tempFramePoint2d.add(this.tempDouble, copOffsets.get(swingSide).get(exitCoPName).getY());
      if (isConstrainedToSupportPolygonFlags.get(exitCoPName))
         constrainInitialCoPPointToSupportPolygon(tempFramePoint2d, exitCoPName);

      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
      framePointToPack.changeFrame(worldFrame);
   }

   private double getInitialStepLengthToCoPOffset(CoPPointName copPointName)
   {
      switch (stepLengthOffsetReferencePolygons.get(copPointName))
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         return getStepLengthBasedOffset(currentSwingFootInitialPolygon, currentSupportFootPolygon, stepLengthToCoPOffsetFactors.get(copPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSwingFootInitialPolygon, currentSupportFootPolygon);
         return getStepLengthBasedOffset(currentSwingFootInitialPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(copPointName));
      case SUPPORT_FOOT_POLYGON:
      default:
         return 0.0;
      }
   }

   private void constrainInitialCoPPointToSupportPolygon(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         constrainToSupportPolygon(tempFramePoint2d, currentSupportFootPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSwingFootInitialPolygon, currentSupportFootPolygon);
         constrainToSupportPolygon(tempFramePoint2d, tempDoubleSupportPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case SUPPORT_FOOT_POLYGON:
      default:
         constrainToSupportPolygon(tempFramePoint2d, currentSwingFootInitialPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
      }
   }

   private void setInitialCoPPointToPolygonOrigin(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to compute initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(currentSupportFootPolygon.getCentroid());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSwingFootInitialPolygon, currentSupportFootPolygon);
         copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case SUPPORT_FOOT_POLYGON:
      default:
         copPointToPlan.setIncludingFrame(currentSwingFootInitialPolygon.getCentroid());
      }
   }

   private void computeCoPPointsForUpcomingFootsteps(int copLocationIndex)
   {
      int numberOfUpcomingFootsteps = Math.min(this.numberOfFootstepsConsidered, this.numberOfUpcomingFootsteps.getIntegerValue());
      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         updateFootPolygons();
         computeCoPPointsForFootstep(copLocationIndex);
         footstepIndex++;
         copLocationIndex++;
      }
      if (copPointList[copPointList.length - 1] != endCoPName)
      {
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         computeMidFeetPointWithChickenSupportForFinalTransfer(tempFramePoint);
         copLocationWaypoints.get(copLocationIndex).addAndSetIncludingFrame(endCoPName, segmentTimes.get(endCoPName), tempFramePoint);
      }
   }

   /**
    * Assumes all the support polygons have been set accordingly and computes the CoP points for the current footstep
    */
   private void computeCoPPointsForFootstep(int copLocationsIndex)
   {
      for (int i = 0; i < copPointList.length; i++)
      {
         computeCoPPointLocation(tempFramePoint2d, copPointList[i], upcomingFootstepsData.get(footstepIndex).getSupportSide());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(copPointList[i], segmentTimes.get(copPointList[i]), tempFramePoint);

      }
   }

   private void computeCoPPointLocation(FramePoint2d copPointToPlan, CoPPointName copPointName, RobotSide supportSide)
   {
      setCoPPointToPolygonOrigin(copPointToPlan, copPointName);
      this.tempDouble = copOffsets.get(supportSide).get(copPointName).getX() + getStepLengthToCoPOffset(copPointName);

      if (isConstrainedToMinMaxFlags.get(copPointName))
         this.tempDouble = MathTools.clamp(this.tempDouble, minCoPOffsets.get(copPointName).getDoubleValue(), maxCoPOffsets.get(copPointName).getDoubleValue());
      copPointToPlan.add(this.tempDouble, copOffsets.get(supportSide).get(copPointName).getY());
      if (isConstrainedToSupportPolygonFlags.get(copPointName))
         constrainCoPPointToSupportPolygon(copPointToPlan, copPointName);
   }

   private double getStepLengthToCoPOffset(CoPPointName coPPointName)
   {
      switch (stepLengthOffsetReferencePolygons.get(coPPointName))
      {
      case INITIAL_SWING_POLYGON:
         return getStepLengthBasedOffset(currentSupportFootPolygon, currentSwingFootInitialPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_SWING_POLYGON:
         return getStepLengthBasedOffset(currentSupportFootPolygon, currentSwingFootFinalPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         return getStepLengthBasedOffset(currentSupportFootPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         return getStepLengthBasedOffset(currentSupportFootPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      default:
         return 0.0;
      }
   }

   private void setCoPPointToPolygonOrigin(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case INITIAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(currentSwingFootInitialPolygon.getCentroid());
         return;
      case FINAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(currentSwingFootFinalPolygon.getCentroid());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case NULL:
         throw new RuntimeException("No frame defined for CoP point:" + copPointName.toString());
      default:
         copPointToPlan.setIncludingFrame(currentSupportFootPolygon.getCentroid());
         return;
      }
   }

   private double getStepLengthBasedOffset(FrameConvexPolygon2d supportPolygon, FrameConvexPolygon2d referencePolygon, double stepLengthToCoPOffsetFactor)
   {
      return stepLengthToCoPOffsetFactor * (referencePolygon.getCentroid().getX() - supportPolygon.getCentroid().getX());
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by projection
    * @param copPointToConstrain
    * @param supportFoot
    * @param safeDistanceFromSupportPolygonEdges
    */
   private void constrainToSupportPolygon(FramePoint2d copPointToConstrain, FrameConvexPolygon2d supportPolygon, double safeDistanceFromSupportPolygonEdges)
   {
      polygonShrinker.shrinkConstantDistanceInto(supportPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      tempPolygon.orthogonalProjection(copPointToConstrain);
   }

   private void constrainCoPPointToSupportPolygon(FramePoint2d copPointToConstrain, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case INITIAL_SWING_POLYGON:
         constrainToSupportPolygon(copPointToConstrain, currentSwingFootInitialPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_SWING_POLYGON:
         constrainToSupportPolygon(copPointToConstrain, currentSwingFootFinalPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         constrainToSupportPolygon(copPointToConstrain, tempDoubleSupportPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         constrainToSupportPolygon(copPointToConstrain, tempDoubleSupportPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case NULL:
         throw new RuntimeException("Invalid constraining frame defined for " + copPointName.toString());
      default:
         constrainToSupportPolygon(copPointToConstrain, currentSupportFootPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         break;
      }
   }

   /**
    * Updates the variable {@code currentDoubleSupportPolygon} from the specified swing and support polygons 
    */
   private void updateDoubleSupportPolygon(FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon)
   {
      tempDoubleSupportPolygon.setIncludingFrame(supportFootPolygon);
      tempDoubleSupportPolygon.changeFrame(worldFrame);
      tempPolygon.setIncludingFrame(swingFootPolygon);
      tempPolygon.changeFrame(worldFrame);
      tempDoubleSupportPolygon.addVertices(tempPolygon);
      tempDoubleSupportPolygon.update();
   }

   /**
    * Updates the swing and support foot polygons based on footstepIndex
    * <p> Has no memory of the previous state so should be used carefully </p>
    */
   private void updateFootPolygons()
   {
      if (footstepIndex == plannedFootstepIndex)
         return;

      switch (footstepIndex)
      {
      case 0:
         initializeFootPolygons();
         break;
      default:
         if (upcomingFootstepsData.get(footstepIndex).getSwingSide() == upcomingFootstepsData.get(footstepIndex - 1).getSupportSide()) // the normal way 
         {
            currentSwingFootInitialPolygon.setIncludingFrame(currentSupportFootPolygon);
            currentSupportFootPolygon.setIncludingFrame(currentSwingFootFinalPolygon);
         }
         else
         {
            currentSwingFootInitialPolygon.setIncludingFrame(currentSwingFootFinalPolygon);
         }
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
      }
      plannedFootstepIndex = footstepIndex;
   }

   /**
    * Initialize the swing and support foot polygons based on footstep index 
    */
   private void initializeFootPolygons()
   {
      currentFootstepData = upcomingFootstepsData.get(footstepIndex);
      switch (footstepIndex)
      {
      case 0:
         setFootPolygonFromCurrentState(currentSwingFootInitialPolygon, currentFootstepData.getSwingSide());
         setFootPolygonFromCurrentState(currentSupportFootPolygon, currentFootstepData.getSupportSide());
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
         break;
      case 1:
         setFootPolygonFromCurrentState(currentSwingFootInitialPolygon, currentFootstepData.getSwingSide());
         setFootPolygonFromFootstep(currentSupportFootPolygon, footstepIndex - 1);
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
         break;
      default:
         setFootPolygonFromFootstep(currentSwingFootInitialPolygon, footstepIndex - 2);
         setFootPolygonFromFootstep(currentSupportFootPolygon, footstepIndex - 1);
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
      }
   }

   private void setFootPolygonFromFootstep(FrameConvexPolygon2d framePolygonToPack, int footstepIndex)
   {
      framePolygonToPack.clear(upcomingFootstepsData.get(footstepIndex).getFootstep().getSoleReferenceFrame());
      framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstepsData.get(footstepIndex).getSwingSide()));
      framePolygonToPack.update();
      framePolygonToPack.changeFrame(worldFrame);
   }

   private void setFootPolygonFromCurrentState(FrameConvexPolygon2d framePolygonToPack, RobotSide robotSide)
   {
      if (!supportFootPolygonsInSoleZUpFrames.get(robotSide).isEmpty())
         framePolygonToPack.setIncludingFrame(supportFootPolygonsInSoleZUpFrames.get(robotSide));
      else
      {
         framePolygonToPack.clear(soleZUpFrames.get(robotSide));
         framePolygonToPack.addVertices(defaultFootPolygons.get(robotSide));
      }
      framePolygonToPack.update();
      framePolygonToPack.changeFrame(worldFrame);
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null && timing != null)
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
         {
            upcomingFootstepsData.add().set(footstep, timing);
            numberOfUpcomingFootsteps.increment();
         }
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
   }

   public void removeFootstepQueueFront()
   {
      removeFootstep(0);
   }

   public void removeFootstepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
         removeFootstep(0);
   }

   public void removeFootstep(int index)
   {
      upcomingFootstepsData.remove(index);
      numberOfUpcomingFootsteps.decrement();
      clearPlan();
   }

   @Override
   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }

   @Override
   public void setSafeDistanceFromSupportEdges(double distance)
   {
      safeDistanceFromCoPToSupportEdges.set(distance);
   }

   @Override
   public List<CoPPointsInFoot> getWaypoints()
   {
      return copLocationWaypoints;
   }

   // TODO This function needs aesthetic improvement
   private void generateCoPTrajectoriesFromWayPoints(CoPTrajectoryType initialTrajectoryType)
   {
      //It is always guaranteed that the initial state will be transfer the way this code is written. This is needed for the angular momentum approximation to work
      CoPTrajectoryType trajectoryType = CoPTrajectoryType.TRANSFER;
      double timeInState = 0.0;
      int transferTrajectoryIndex = -1;
      int swingTrajectoryIndex = -1;
      CoPSplineType splineInterpolationOrder = orderOfSplineInterpolation.getEnumValue();
      tempFramePoint.setToNaN();
      for (int waypointIndex = 0; waypointIndex < copLocationWaypoints.size() && !copLocationWaypoints.get(waypointIndex).getCoPPointList().isEmpty(); waypointIndex++)
      {
         List<CoPPointName> copList = copLocationWaypoints.get(waypointIndex).getCoPPointList();
         for (int segmentIndex = 0; segmentIndex < copList.size(); segmentIndex++)
         {
            CoPTrajectoryPoint currentPoint = copLocationWaypoints.get(waypointIndex).get(copList.get(segmentIndex));
            if (!tempFramePoint.containsNaN())
            {
               if (trajectoryType == CoPTrajectoryType.SWING)
                  swingCoPTrajectories.get(swingTrajectoryIndex).setSegment(splineInterpolationOrder, timeInState, timeInState + currentPoint.getTime(),
                                                                            tempFramePoint, currentPoint.getPosition().getFrameTuple());
               else
                  transferCoPTrajectories.get(transferTrajectoryIndex).setSegment(splineInterpolationOrder, timeInState, timeInState + currentPoint.getTime(),
                                                                                  tempFramePoint, currentPoint.getPosition().getFrameTuple());
            }
            else
            {
               transferTrajectoryIndex++;
               currentPoint.getPosition(tempFramePoint);
               continue;
            }
            currentPoint.getPosition(tempFramePoint);

            if (copList.get(segmentIndex) == entryCoPName)
            {
               trajectoryType = CoPTrajectoryType.SWING;
               timeInState = 0.0;
               swingTrajectoryIndex++;
            }
            else if (copList.get(segmentIndex) == exitCoPName)
            {
               trajectoryType = CoPTrajectoryType.TRANSFER;
               timeInState = 0.0;
               transferTrajectoryIndex++;
            }
            else
               timeInState += currentPoint.getTime();
         }
      }
//      PrintTools.debug("********************************New Call********************************");
//      for (int i = 0; i < transferCoPTrajectories.size(); i++)
//      {
//         PrintTools.debug("Transfer # Segments: " + transferCoPTrajectories.get(i).getNumberOfSegments());
//         for (int j = 0; j < transferCoPTrajectories.get(i).getNumberOfSegments(); j++)
//         {
//            transferCoPTrajectories.get(i).getPolynomials().get(j).compute(transferCoPTrajectories.get(i).getPolynomials().get(j).getInitialTime());
//            PrintTools.debug("Transfer Trajectory " + i + " InitialPosition: " + transferCoPTrajectories.get(i).getPolynomials().get(j).getPosition());
//            transferCoPTrajectories.get(i).getPolynomials().get(j).compute(transferCoPTrajectories.get(i).getPolynomials().get(j).getFinalTime());
//            PrintTools.debug("Transfer Trajectory " + i + " FinalPosition: " + transferCoPTrajectories.get(i).getPolynomials().get(j).getPosition().toString());
//         }
//      }
//      for (int i = 0; i < swingCoPTrajectories.size(); i++)
//      {
//         PrintTools.debug("Swing # Segments: " + swingCoPTrajectories.get(i).getNumberOfSegments());
//         for (int j = 0; j < swingCoPTrajectories.get(i).getNumberOfSegments(); j++)
//         {
//            swingCoPTrajectories.get(i).getPolynomials().get(j).compute(swingCoPTrajectories.get(i).getPolynomials().get(j).getInitialTime());
//            PrintTools.debug("Swing Trajectory " + i + " InitialPosition: " + swingCoPTrajectories.get(i).getPolynomials().get(j).getPosition().toString());
//            swingCoPTrajectories.get(i).getPolynomials().get(j).compute(swingCoPTrajectories.get(i).getPolynomials().get(j).getFinalTime());
//            PrintTools.debug("Swing Trajectory " + i + " FinalPosition: " + swingCoPTrajectories.get(i).getPolynomials().get(j).getPosition().toString());
//         }
//      }
   }

   @Override
   public List<? extends CoPTrajectoryInterface> getTransferCoPTrajectories()
   {
      return transferCoPTrajectories;
   }

   @Override
   public List<? extends CoPTrajectoryInterface> getSwingCoPTrajectories()
   {
      return swingCoPTrajectories;
   }

}
