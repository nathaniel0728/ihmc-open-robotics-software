package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator.JointControlBlender;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class NewStandTransitionControllerState extends NewHighLevelControllerState
{
   private static final double TIME_TO_RAMP_UP_CONTROL = 0.7;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble standTransitionDuration = new YoDouble("standTransitionDuration", registry);
   private final YoPolynomial walkingControlRatioTrajectory = new YoPolynomial("walkingControlRatioTrajectory", 2, registry);

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);

   private final PairList<OneDoFJoint, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final NewStandReadyControllerState standReadyControllerState;
   private final NewWalkingControllerState walkingControllerState;

   public NewStandTransitionControllerState(NewStandReadyControllerState standReadyControllerState, NewWalkingControllerState walkingControllerState,
                                            HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this(standReadyControllerState, walkingControllerState, controllerToolbox, TIME_TO_RAMP_UP_CONTROL);
   }

   public NewStandTransitionControllerState(NewStandReadyControllerState standReadyControllerState, NewWalkingControllerState walkingControllerState,
                                            HighLevelHumanoidControllerToolbox controllerToolbox, double standTransitionDuration)
   {
      super(NewHighLevelControllerStates.STAND_TRANSITION_STATE);

      this.standReadyControllerState = standReadyControllerState;
      this.walkingControllerState = walkingControllerState;
      this.standTransitionDuration.set(standTransitionDuration);

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         JointControlBlender jointControlBlender = new JointControlBlender("_StandTransition", controlledJoint, registry);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      walkingControllerState.doTransitionIntoAction();

      walkingControlRatioTrajectory.setLinear(0.0, standTransitionDuration.getDoubleValue(), 0.0, 1.0);
   }

   @Override
   public void doAction()
   {
      standReadyControllerState.doAction();
      walkingControllerState.doAction();

      walkingControlRatioTrajectory.compute(getTimeInCurrentState());
      double gainRatio = walkingControlRatioTrajectory.getPosition();

      ControllerCoreCommand standReadyCommand = standReadyControllerState.getControllerCoreCommand();
      ControllerCoreCommand walkingCommand = walkingControllerState.getControllerCoreCommand();
      LowLevelOneDoFJointDesiredDataHolder standReadyJointCommand = standReadyCommand.getLowLevelOneDoFJointDesiredDataHolder();
      LowLevelOneDoFJointDesiredDataHolder walkingJointCommand = walkingCommand.getLowLevelOneDoFJointDesiredDataHolder();

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJoint joint = jointCommandBlenders.get(jointIndex).getLeft();
         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         LowLevelJointData lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);

         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData, standReadyJointCommand.getLowLevelJointData(joint),
                                                          walkingJointCommand.getLowLevelJointData(joint), gainRatio);
      }

      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      standReadyControllerState.doTransitionOutOfAction();
   }

   @Override
   public boolean isDone()
   {
      return getTimeInCurrentState() > standTransitionDuration.getDoubleValue();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

}