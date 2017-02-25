package us.ihmc.atlas.StepAdjustmentVisualizers;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

import java.awt.*;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class StepAdjustmentExampleGraphic
{
   private static final double controlDT = 0.001;

   private static final double footLengthForControl = 0.22;
   private static final double footWidthForControl = 0.0825;
   private static final double toeWidthForControl = 0.11;

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final YoVariableRegistry registry;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private ReferenceFrame midFeetZUpFrame;

   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<FramePose>(new FramePose(), new FramePose());
   private final SideDependentList<YoFramePose> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private final DoubleYoVariable doubleSupportDuration;
   private final DoubleYoVariable singleSupportDuration;
   private final DoubleYoVariable omega0;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable timeToConsiderAdjustment;

   private final YoFramePoint2d yoDesiredCMP;
   private final YoFramePoint2d yoCurrentICP;
   private final YoFramePoint2d yoDesiredICP;

   private final ArrayList<Footstep> footsteps = new ArrayList<>();
   private final ArrayList<Footstep> nextFootsteps = new ArrayList<>();

   private final YoFramePose yoNextFootstepPose;
   private final YoFramePose yoNextNextFootstepPose;
   private final YoFramePose yoNextNextNextFootstepPose;
   private final YoFrameConvexPolygon2d yoNextFootstepPolygon;
   private final YoFrameConvexPolygon2d yoNextNextFootstepPolygon;
   private final YoFrameConvexPolygon2d yoNextNextNextFootstepPolygon;

   private BipedSupportPolygons bipedSupportPolygons;
   private FootstepTestHelper footstepTestHelper;

   private final CapturePointPlannerParameters capturePointPlannerParameters;
   private final ICPOptimizationParameters icpOptimizationParameters;
   private final ICPOptimizationController icpOptimizationController;
   private final ICPPlanner icpPlanner;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SimulationConstructionSet scs;

   public StepAdjustmentExampleGraphic()
   {
      Robot robot = new DummyRobot();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      registry = robot.getRobotsYoVariableRegistry();

      WalkingControllerParameters walkingControllerParameters = createWalkingControllerParamters();
      capturePointPlannerParameters = createICPPlannerParameters();
      icpOptimizationParameters = createICPOptimizationParameters();

      yoDesiredCMP = new YoFramePoint2d("desiredCMP", worldFrame, registry);
      yoCurrentICP = new YoFramePoint2d("currentICP", worldFrame, registry);
      yoDesiredICP = new YoFramePoint2d("desiredICP", worldFrame, registry);

      doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
      singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
      omega0 = new DoubleYoVariable("omega0", registry);
      doubleSupportDuration.set(0.25);
      singleSupportDuration.set(0.75);
      omega0.set(3.0);

      yoTime = robot.getYoTime();
      timeToConsiderAdjustment = new DoubleYoVariable("timeToConsiderAdjustment", registry);
      timeToConsiderAdjustment.set(0.5 * singleSupportDuration.getDoubleValue());

      setupFeetFrames(yoGraphicsListRegistry);

      icpPlanner = new ICPPlanner(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      icpPlanner.setOmega0(omega0.getDoubleValue());
      icpPlanner.setDesiredCapturePointState(new FramePoint2d(ReferenceFrame.getWorldFrame()), new FrameVector2d(ReferenceFrame.getWorldFrame()));

      icpOptimizationController = new ICPOptimizationController(capturePointPlannerParameters, icpOptimizationParameters, walkingControllerParameters, bipedSupportPolygons,
            contactableFeet, controlDT, registry, yoGraphicsListRegistry);

      yoNextFootstepPose = new YoFramePose("nextFootstepPose", worldFrame, registry);
      yoNextNextFootstepPose = new YoFramePose("nextNextFootstepPose", worldFrame, registry);
      yoNextNextNextFootstepPose = new YoFramePose("nextNextNextFootstepPose", worldFrame, registry);
      yoNextFootstepPolygon = new YoFrameConvexPolygon2d("nextFootstep", "", worldFrame, 4, registry);
      yoNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextFootstep", "", worldFrame, 4, registry);
      yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextNextFootstep", "", worldFrame, 4, registry);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<>();
      for (FramePoint2d point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(point.getPointCopy());
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));

      YoGraphicShape nextFootstepViz = new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0);
      YoGraphicShape nextNextFootstepViz = new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0);
      YoGraphicShape nextNextNextFootstepViz = new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0);

      yoGraphicsListRegistry.registerYoGraphic("dummy", nextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic("dummy", nextNextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic("dummy", nextNextNextFootstepViz);

      scs = new SimulationConstructionSet(robot);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      Thread myThread = new Thread(scs);
      myThread.start();

      initialize();
   }

   private void setupFeetFrames(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         currentFootPoses.put(robotSide, new YoFramePose(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet, ankleFrames);

   }

   private final FootstepTiming timing = new FootstepTiming();
   private void initialize()
   {
      for (Footstep footstep : footstepTestHelper.createFootsteps(0.4, 0.6, 3))
      {
         footsteps.add(footstep);
      }

      icpPlanner.clearPlan();
      icpOptimizationController.clearPlan();

      Footstep nextFootstep = footsteps.get(0);
      Footstep nextNextFootstep = footsteps.get(0);
      Footstep nextNextNextFootstep = footsteps.get(1);

      nextFootsteps.add(nextFootstep);
      nextFootsteps.add(nextNextFootstep);
      nextFootsteps.add(nextNextNextFootstep);

      for (int i = 3; i < icpOptimizationController.getNumberOfFootstepsToConsider(); i++)
         nextFootsteps.add(footsteps.get(i - 1));

      timing.setTimings(doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      icpPlanner.addFootstepToPlan(nextFootstep, timing);
      icpPlanner.addFootstepToPlan(nextNextFootstep, timing);
      icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing);



      icpOptimizationController.setStepDurations(doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      icpOptimizationController.addFootstepToPlan(nextFootstep);
      icpOptimizationController.addFootstepToPlan(nextNextFootstep);
      icpOptimizationController.addFootstepToPlan(nextNextNextFootstep);

      RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

      icpPlanner.setSupportLeg(supportSide);
      icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

      icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide, omega0.getDoubleValue());

      FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
      FramePose nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
      nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
      nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
      footSpoof.setSoleFrame(nextSupportPose);

      contactStates.get(supportSide.getOppositeSide()).clear();
      if (nextFootstep.getPredictedContactPoints() == null)
         contactStates.get(supportSide.getOppositeSide()).setContactFramePoints(footSpoof.getContactPoints2d());
      else
         contactStates.get(supportSide.getOppositeSide()).setContactPoints(nextFootstep.getPredictedContactPoints());

      updateViz(false);

   }

   private final FramePose footstepPose = new FramePose();
   private final FramePoint2d footstepPositionSolution = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FramePoint2d desiredICP = new FramePoint2d();
   private final FrameVector2d desiredICPVelocity = new FrameVector2d();
   private final FramePoint2d currentICP = new FramePoint2d();

   public void update()
   {
      yoCurrentICP.getFrameTuple2d(currentICP);

      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredICP, desiredICPVelocity, timeToConsiderAdjustment.getDoubleValue());
      yoDesiredICP.set(desiredICP);

      icpOptimizationController.compute(timeToConsiderAdjustment.getDoubleValue(), desiredICP, desiredICPVelocity, currentICP, omega0.getDoubleValue());
      icpOptimizationController.getDesiredCMP(desiredCMP);
      yoDesiredCMP.set(desiredCMP);

      for (int i = 0; i < icpOptimizationController.getNumberOfFootstepsToConsider(); i++)
      {
         Footstep footstep = nextFootsteps.get(i);

         if (footstep != null)
         {
            footstep.getPose(footstepPose);
            icpOptimizationController.getFootstepSolution(i, footstepPositionSolution);
            footstepPose.setXYFromPosition2d(footstepPositionSolution);
            footstep.setPose(footstepPose);
         }
      }

      updateViz(false);
   }

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   private void updateViz(boolean isInTransfer)
   {
      Footstep nextFootstep = nextFootsteps.get(0);
      Footstep nextNextFootstep = nextFootsteps.get(1);
      Footstep nextNextNextFootstep = nextFootsteps.get(2);

      if (nextFootstep == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.hide();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextFootstep.getPredictedContactPoints() == null)
         nextFootstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(nextFootstep.getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextFootstep.getSoleReferenceFrame(), nextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextFootstepPose = new FramePose(nextFootstep.getSoleReferenceFrame());
      yoNextFootstepPose.setAndMatchFrame(nextFootstepPose);

      if (nextNextFootstep == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextNextFootstep.getPredictedContactPoints() == null)
         nextNextFootstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(nextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextNextFootstep.getSoleReferenceFrame(), nextNextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextFootstepPose = new FramePose(nextNextFootstep.getSoleReferenceFrame());
      yoNextNextFootstepPose.setAndMatchFrame(nextNextFootstepPose);

      if (nextNextNextFootstep == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextNextNextFootstep.getPredictedContactPoints() == null)
         nextNextNextFootstep.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(nextNextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextNextNextFootstep.getSoleReferenceFrame(), nextNextNextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextNextFootstepPose = new FramePose(nextNextNextFootstep.getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setAndMatchFrame(nextNextNextFootstepPose);

      if (!isInTransfer)
      {
         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
         FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
         FramePose nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
         nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
         nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
         footSpoof.setSoleFrame(nextSupportPose);
      }
   }



   public static void main(String[] args)
   {
      StepAdjustmentExampleGraphic stepAdjustmentExampleGraphic = new StepAdjustmentExampleGraphic();
   }

   private class DummyRobot extends  Robot
   {
      public DummyRobot()
      {
         super("dummyRobot");
      }

      @Override
      public void update()
      {
         super.update();
      }

   }

   private WalkingControllerParameters createWalkingControllerParamters()
   {
      return new WalkingControllerParameters()
      {
         @Override
         public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
         {
            return null;
         }

         @Override
         public String[] getDefaultChestOrientationControlJointNames()
         {
            return new String[0];
         }

         @Override
         public double getOmega0()
         {
            return 0;
         }

         @Override
         public double getAnkleHeight()
         {
            return 0;
         }

         @Override
         public double getLegLength()
         {
            return 0;
         }

         @Override
         public double getMinLegLengthBeforeCollapsingSingleSupport()
         {
            return 0;
         }

         @Override
         public double getMinMechanicalLegLength()
         {
            return 0;
         }

         @Override
         public double minimumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double nominalHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double maximumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double defaultOffsetHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double pelvisToAnkleThresholdForWalking()
         {
            return 0;
         }

         @Override
         public double getTimeToGetPreparedForLocomotion()
         {
            return 0;
         }

         @Override
         public boolean doToeOffIfPossible()
         {
            return false;
         }

         @Override
         public boolean doToeOffIfPossibleInSingleSupport()
         {
            return false;
         }

         @Override
         public boolean checkECMPLocationToTriggerToeOff()
         {
            return false;
         }

         @Override
         public double getMinStepLengthForToeOff()
         {
            return 0;
         }

         @Override
         public boolean doToeOffWhenHittingAnkleLimit()
         {
            return false;
         }

         @Override
         public double getMaximumToeOffAngle()
         {
            return 0;
         }

         @Override
         public boolean doToeTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getToeTouchdownAngle()
         {
            return 0;
         }

         @Override
         public boolean doHeelTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getHeelTouchdownAngle()
         {
            return 0;
         }

         @Override
         public boolean allowShrinkingSingleSupportFootPolygon()
         {
            return false;
         }

         @Override
         public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
         {
            return false;
         }

         @Override
         public boolean allowAutomaticManipulationAbort()
         {
            return false;
         }

         @Override
         public double getMinimumSwingTimeForDisturbanceRecovery()
         {
            return 0;
         }

         @Override
         public boolean useOptimizationBasedICPController()
         {
            return false;
         }

         @Override
         public double getICPErrorThresholdToSpeedUpSwing()
         {
            return 0;
         }

         @Override
         public ICPControlGains createICPControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public boolean getCoMHeightDriftCompensation()
         {
            return false;
         }

         @Override
         public YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createEdgeTouchdownFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public double getSwingHeightMaxForPushRecoveryTrajectory()
         {
            return 0;
         }

         @Override
         public boolean doPrepareManipulationForLocomotion()
         {
            return false;
         }

         @Override
         public boolean controlHeadAndHandsWithSliders()
         {
            return false;
         }

         @Override
         public double getDefaultTransferTime()
         {
            return 0;
         }

         @Override
         public double getDefaultSwingTime()
         {
            return 0;
         }

         @Override
         public double getSpineYawLimit()
         {
            return 0;
         }

         @Override
         public double getSpinePitchUpperLimit()
         {
            return 0;
         }

         @Override
         public double getSpinePitchLowerLimit()
         {
            return 0;
         }

         @Override
         public double getSpineRollLimit()
         {
            return 0;
         }

         @Override
         public boolean isSpinePitchReversed()
         {
            return false;
         }

         @Override
         public double getFoot_start_toetaper_from_back()
         {
            return 0;
         }

         @Override
         public double getSideLengthOfBoundingBoxForFootstepHeight()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownHeightOffset()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownVelocity()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownAcceleration()
         {
            return 0;
         }

         @Override
         public double getContactThresholdForce()
         {
            return 0;
         }

         @Override
         public double getSecondContactThresholdForceIgnoringCoP()
         {
            return 0;
         }

         @Override
         public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
         {
            return null;
         }

         @Override
         public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
         {
            return null;
         }

         @Override
         public double getCoPThresholdFraction()
         {
            return 0;
         }

         @Override
         public String[] getJointsToIgnoreInController()
         {
            return new String[0];
         }

         @Override
         public MomentumOptimizationSettings getMomentumOptimizationSettings()
         {
            return null;
         }

         @Override
         public boolean doFancyOnToesControl()
         {
            return false;
         }

         @Override
         public FootSwitchType getFootSwitchType()
         {
            return null;
         }

         @Override
         public double getContactThresholdHeight()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportX()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportY()
         {
            return 0;
         }

         @Override
         public boolean finishSingleSupportWhenICPPlannerIsDone()
         {
            return false;
         }

         @Override
         public void useInverseDynamicsControlCore()
         {

         }

         @Override
         public void useVirtualModelControlCore()
         {

         }

         @Override
         public double getHighCoPDampingDurationToPreventFootShakies()
         {
            return 0;
         }

         @Override
         public double getCoPErrorThresholdForHighCoPDamping()
         {
            return 0;
         }

         @Override
         public double getMaxStepLength()
         {
            return 0;
         }

         @Override
         public double getDefaultStepLength()
         {
            return 0;
         }

         @Override
         public double getMaxStepWidth()
         {
            return 0;
         }

         @Override
         public double getInPlaceWidth()
         {
            return 0;
         }

         @Override
         public double getDesiredStepForward()
         {
            return 0;
         }

         @Override
         public double getMaxStepUp()
         {
            return 0;
         }

         @Override
         public double getMaxSwingHeightFromStanceFoot()
         {
            return 0;
         }

         @Override
         public double getMaxAngleTurnOutwards()
         {
            return 0;
         }

         @Override
         public double getMaxAngleTurnInwards()
         {
            return 0;
         }

         @Override
         public double getMinAreaPercentForValidFootstep()
         {
            return 0;
         }

         @Override
         public double getDangerAreaPercentForValidFootstep()
         {
            return 0;
         }

         @Override
         public double getFootForwardOffset()
         {
            return 0;
         }

         @Override
         public double getFootBackwardOffset()
         {
            return 0;
         }

         @Override
         public double getFootWidth()
         {
            return 0;
         }

         @Override
         public double getToeWidth()
         {
            return 0;
         }

         @Override
         public double getFootLength()
         {
            return 0;
         }

         @Override
         public double getActualFootWidth()
         {
            return 0;
         }

         @Override
         public double getActualFootLength()
         {
            return 0;
         }

         @Override
         public double getFootstepArea()
         {
            return 0;
         }

         @Override
         public String[] getDefaultHeadOrientationControlJointNames()
         {
            return new String[0];
         }

         @Override
         public YoOrientationPIDGainsInterface createHeadOrientationControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoPIDGains createHeadJointspaceControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public double[] getInitialHeadYawPitchRoll()
         {
            return new double[0];
         }

         @Override
         public boolean isNeckPositionControlled()
         {
            return false;
         }

         @Override
         public double getNeckPitchUpperLimit()
         {
            return 0;
         }

         @Override
         public double getNeckPitchLowerLimit()
         {
            return 0;
         }

         @Override
         public double getHeadYawLimit()
         {
            return 0;
         }

         @Override
         public double getHeadRollLimit()
         {
            return 0;
         }

         @Override
         public double getTrajectoryTimeHeadOrientation()
         {
            return 0;
         }

         @Override
         public double getMinStepWidth()
         {
            return 0;
         }

         @Override
         public double getStepPitch()
         {
            return 0;
         }

         @Override
         public double getMaxStepDown()
         {
            return 0;
         }
      };
   }

   private CapturePointPlannerParameters createICPPlannerParameters()
   {
      return new CapturePointPlannerParameters()
      {
         @Override
         public double getDoubleSupportSplitFraction()
         {
            return 0.5;
         }

         @Override
         public double getEntryCMPInsideOffset()
         {
            return -0.005; // 0.006;
         }

         @Override
         public double getExitCMPInsideOffset()
         {
            return 0.025;
         }

         @Override
         public double getEntryCMPForwardOffset()
         {
            return 0.0;
         }

         @Override
         public double getExitCMPForwardOffset()
         {
            return 0.0;
         }

         @Override
         public boolean useTwoCMPsPerSupport()
         {
            return true;
         }

         @Override
         public double getTimeSpentOnExitCMPInPercentOfStepTime()
         {
            return 0.5;
         }

         @Override
         public double getMaxEntryCMPForwardOffset()
         {
            return 0.03;
         }

         @Override
         public double getMinEntryCMPForwardOffset()
         {
            return -0.05;
         }

         @Override
         public double getMaxExitCMPForwardOffset()
         {
            return 0.15;
         }

         @Override
         public double getMinExitCMPForwardOffset()
         {
            return -0.04;
         }

         @Override
         public double getCMPSafeDistanceAwayFromSupportEdges()
         {
            return 0.001;
         }

         @Override
         public double getMaxDurationForSmoothingEntryToExitCMPSwitch()
         {
            return 1.0;
         }

         /** {@inheritDoc} */
         @Override
         public boolean useExitCMPOnToesForSteppingDown()
         {
            return true;
         }
      };
   }

   public ICPOptimizationParameters createICPOptimizationParameters()
   {
      return new ICPOptimizationParameters()
      {
         @Override public int getMaximumNumberOfFootstepsToConsider()
         {
            return 5;
         }

         @Override
         public int numberOfFootstepsToConsider()
         {
            return 1;
         }

         @Override public double getFootstepRegularizationWeight()
         {
            return 0.01;
         }

         @Override public double getFeedbackRegularizationWeight()
         {
            return 0.0001;
         }

         @Override public double getForwardFootstepWeight()
         {
            return 100.0;
         }

         @Override public double getLateralFootstepWeight()
         {
            return 100.0;
         }

         @Override public double getFeedbackForwardWeight()
         {
            return 0.1;
         }

         @Override public double getFeedbackLateralWeight()
         {
            return 0.1;
         }

         @Override
         public boolean useDifferentSplitRatioForBigAdjustment()
         {
            return true;
         }

         @Override
         public double getMagnitudeForBigAdjustment()
         {
            return 0.2;
         }

         @Override public double getFeedbackParallelGain()
         {
            return 6.0;
         }

         @Override public double getFeedbackOrthogonalGain()
         {
            return 6.0;
         }

         @Override public double getDynamicRelaxationWeight()
         {
            return 1000.0;
         }

         @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
         {
            return 5.0;
         }

         @Override public boolean useFeedbackRegularization()
         {
            return true;
         }

         @Override public boolean useStepAdjustment()
         {
            return true;
         }

         @Override public boolean useFootstepRegularization()
         {
            return true;
         }

         @Override public boolean scaleStepRegularizationWeightWithTime()
         {
            return true;
         }

         @Override public boolean scaleUpcomingStepWeights()
         {
            return true;
         }

         @Override public boolean scaleFeedbackWeightWithGain()
         {
            return true;
         }

         @Override public double getMinimumFootstepWeight()
         {
            return 0.0001;
         }

         @Override public double getMinimumFeedbackWeight()
         {
            return 0.0001;
         }

         @Override public double getMinimumTimeRemaining()
         {
            return 0.001;
         }

         @Override
         public double getDoubleSupportMaxCMPForwardExit()
         {
            return 0;
         }

         @Override
         public double getDoubleSupportMaxCMPLateralExit()
         {
            return 0;
         }

         @Override
         public double getSingleSupportMaxCMPForwardExit()
         {
            return 0;
         }

         @Override
         public double getSingleSupportMaxCMPLateralExit()
         {
            return 0;
         }

         @Override public double getAdjustmentDeadband()
         {
            return 0.02;
         }
      };
   }
}