package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import org.apache.bcel.generic.CPInstruction;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CoPPolynomialTrajectoryPlannerInterface;
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
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
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

public class ReferenceCoPTrajectoryGenerator implements CoPPolynomialTrajectoryPlannerInterface, ReferenceCoPTrajectoryGeneratorInterface
{
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double COP_POINT_SIZE = 0.005;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix;

   // Waypoint planning parameters
   private final EnumMap<CoPPointName, YoDouble> maxCoPOffsets = new EnumMap<CoPPointName, YoDouble>(CoPPointName.class);
   private final EnumMap<CoPPointName, YoDouble> minCoPOffsets = new EnumMap<CoPPointName, YoDouble>(CoPPointName.class);
   private final SideDependentList<EnumMap<CoPPointName, YoFrameVector2d>> copUserOffsets = new SideDependentList<>();
   private final YoDouble safeDistanceFromCoPToSupportEdges;
   private final YoDouble stepLengthToCoPOffsetFactor;
   private final YoDouble percentageChickenSupport;

   // Trajectory planning parameters
   private final List<YoDouble> swingDurations;
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> swingDurationShiftFractions;
   private final List<YoDouble> transferDurations;
   private final List<YoDouble> transferSplitFractions;

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
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FramePoint desiredCoPPosition = new FramePoint();
   private FrameVector desiredCoPVelocity = new FrameVector();
   private FrameVector desiredCoPAcceleration = new FrameVector();
   private int numberOfFootstepsConsidered;  // Always sync this with the YoInteger numberOfFootstepsToConsider
   private int numberOfCoPPointsPerFoot;     // Always sync this with the YoInteger numberOfPointsPerFoot

   // Planner level overrides (the planner knows better!)
   private static final int maxNumberOfPointsInFoot = 4;
   private static final int maxNumberOfFootstepsToConsider = 4;

   // Input data
   private final RecyclingArrayList<FootstepData> upcomingFootstepsData = new RecyclingArrayList<FootstepData>(maxNumberOfFootstepsToConsider, FootstepData.class);
   private final YoInteger numberOfUpcomingFootsteps;

   public ReferenceCoPTrajectoryGenerator(String namePrefix, SmoothCMPPlannerParameters plannerParameters, BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberOfFootstepsToConsider,
                                          List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> swingSplitFractions,
                                          List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;

      EnumMap<CoPPointName, Double> maxCoPOffsets = plannerParameters.getMaxCoPForwardOffsetsFootFrame();
      EnumMap<CoPPointName, Double>  minCoPOffsets = plannerParameters.getMinCoPForwardOffsetsFootFrame();

      for (CoPPointName copPointName : plannerParameters.getCoPPointsToPlan())
      {         
         YoDouble maxCoPOffset = new YoDouble("maxCoPForwardOffset" + copPointName.toString(), registry);
         YoDouble minCoPOffset = new YoDouble("minCoPForwardOffset" + copPointName.toString(), registry);
         maxCoPOffset.set(maxCoPOffsets.get(copPointName));
         minCoPOffset.set(minCoPOffsets.get(copPointName));
         this.maxCoPOffsets.put(copPointName, maxCoPOffset);
         this.minCoPOffsets.put(copPointName, minCoPOffset);
      }
      safeDistanceFromCoPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdges", registry);
      stepLengthToCoPOffsetFactor = new YoDouble(namePrefix + "StepLengthToCMPOffsetFactor", registry);

      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
      percentageChickenSupport.set(0.5);
            
      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      for (RobotSide robotSide : RobotSide.values)
      {
         supportFootPolygonsInSoleZUpFrames.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(robotSide));
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         EnumMap<CoPPointName, YoFrameVector2d> copUserOffsets = new EnumMap<>(CoPPointName.class);
         
         for(CoPPointName copPointName : plannerParameters.getCoPPointsToPlan())
         {
            YoFrameVector2d copUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "CoPConstantOffset" + copPointName.toString(), null, registry);
            copUserOffsets.put(copPointName, copUserOffset);
         }
         this.copUserOffsets.put(robotSide, copUserOffsets);
      }
      
      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      setNumberOfPointsPerFoot(plannerParameters.getNumberOfWayPointsPerFoot());

      this.numberOfFootstepsToConsider = new YoInteger(namePrefix + "NumberOfFootstepsToConsider", registry);
      setNumberOfFootstepsToConsider(plannerParameters.getNumberOfFootstepsToConsider());
      
      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.orderOfSplineInterpolation.set(plannerParameters.getOrderOfCoPInterpolation());

      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT)};
      // Need two additional CoPPoints in Foot to store the initial and final footstep CoPs
      copLocationWaypoints.add(new CoPPointsInFoot(0, framesToRegister, parentRegistry));
      for (int i = 1 ; i <= this.numberOfFootstepsConsidered; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(i, framesToRegister, registry));
      }
      copLocationWaypoints.add(new CoPPointsInFoot(this.numberOfFootstepsConsidered + 1, framesToRegister, parentRegistry));
      
      for (int i = 0; i < this.numberOfFootstepsConsidered; i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, i, transferDurations.get(i), transferSplitFractions.get(i), registry);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(namePrefix, i, swingDurations.get(i), swingSplitFractions.get(i), swingDurationShiftFractions.get(i), registry);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, numberOfFootstepsConsidered, transferDurations.get(numberOfFootstepsConsidered), transferSplitFractions.get(numberOfFootstepsConsidered), registry);
      transferCoPTrajectories.add(transferCoPTrajectory);

      this.numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfUpcomingFootsteps.set(0);
      parentRegistry.addChild(registry);           
      initializeParameters(plannerParameters);
      clear();
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
      // TODO modify to have things that should be adjusted on the fly here
      safeDistanceFromCoPToSupportEdges.set(parameters.getCoPSafeDistanceAwayFromSupportEdges());
      stepLengthToCoPOffsetFactor.set(parameters.getStepLengthToCoPOffsetFactor());

      EnumMap<CoPPointName, Vector2D> copOffsets = parameters.getCoPOffsetsFootFrame();
      for(CoPPointName copPointName : parameters.getCoPPointsToPlan())
         setSymmetricCoPConstantOffsets(copPointName, copOffsets.get(copPointName));
   }

   @Override
   public void setSymmetricCoPConstantOffsets(CoPPointName copPointName, Vector2D heelOffset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d copUserOffset = copUserOffsets.get(robotSide).get(copPointName);
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
         
         for (CoPPointName copPointName : copPointsInFoot.getCoPPointList())
         {
            YoGraphicPosition copViz = new YoGraphicPosition(footIndex + "Foot CoP Waypoint" + copPointName.toString(),
                                                             copPointsInFoot.getWaypointInWorldFrameReadOnly(copPointName), COP_POINT_SIZE,
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
      
      for (int i = 0; i < numberOfFootstepsConsidered; i++)
      {
         copLocationWaypoints.get(i).reset();
         transferCoPTrajectories.get(i).reset();
         swingCoPTrajectories.get(i).reset();
      }
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
      if(activeTrajectory != null)
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
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void removeFootstepQueueFront()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void removeFootstepQueueFront(int numberOfFootstepsToRemove)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void removeFootstep(int index)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean isDoneWalking()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void setSafeDistanceFromSupportEdges(double distance)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public List<CoPPointsInFoot> getWaypoints()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<? extends CoPTrajectory> getTransferCoPTrajectories()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<? extends CoPTrajectory> getSwingCoPTrajectories()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setInitialCoPPosition(FramePoint initialCoPPosition)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setInitialCoPVelocity(FrameVector2d intialCoPVelocity)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setInitialCoPVelocity(FrameVector intialCoPVelocity)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setInitialCoPAcceleration(FrameVector2d initialCoPAcceleration)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setInitialCoPAcceleration(FrameVector initialCoPAcceleration)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setFinalCoPVelocity(FrameVector finalCoPVelocity)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setFinalCoPVelocity(FrameVector2d finalCoPVelocity)
   {
      // TODO Auto-generated method stub
      
   }

}
