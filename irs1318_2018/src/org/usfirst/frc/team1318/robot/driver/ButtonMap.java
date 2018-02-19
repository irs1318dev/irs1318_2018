package org.usfirst.frc.team1318.robot.driver;

import java.util.HashMap;
import java.util.Map;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.driver.common.IButtonMap;
import org.usfirst.frc.team1318.robot.driver.common.UserInputDeviceButton;
import org.usfirst.frc.team1318.robot.driver.common.buttons.AnalogAxis;
import org.usfirst.frc.team1318.robot.driver.common.buttons.ButtonType;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.AnalogOperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.DigitalOperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.MacroOperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.OperationDescription;
import org.usfirst.frc.team1318.robot.driver.common.descriptions.UserInputDevice;
import org.usfirst.frc.team1318.robot.driver.controltasks.ElevatorMovementTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.EnableWinchTimedTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.IntakeAndCorrectionTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.OuttakeTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.PIDBrakeTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.ReleaseServoTimedTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.SequentialTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.VisionAdvanceAndCenterTask;
import org.usfirst.frc.team1318.robot.driver.controltasks.VisionCenteringTask;

@Singleton
public class ButtonMap implements IButtonMap
{
    @SuppressWarnings("serial")
    public static Map<Operation, OperationDescription> OperationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            put(
                Operation.DebugShift,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TRIGGER_BUTTON,
                    ButtonType.Simple));

            // Operations for vision
            put(
                Operation.EnableVision,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    90, // POV right
                    ButtonType.Toggle));

            // Operations for the drive train
            put(
                Operation.DriveTrainDisablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
                    ButtonType.Click));
            put(
                Operation.DriveTrainEnablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
                    ButtonType.Click));
            put(
                Operation.DriveTrainMoveForward,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.Y,
                    ElectronicsConstants.INVERT_Y_AXIS,
                    TuningConstants.DRIVETRAIN_Y_DEAD_ZONE));
            put(
                Operation.DriveTrainTurn,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.X,
                    ElectronicsConstants.INVERT_X_AXIS,
                    TuningConstants.DRIVETRAIN_X_DEAD_ZONE));
            put(
                Operation.DriveTrainSimpleMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUsePositionalMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUseBrakeMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainLeftPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainSwapFrontOrientation,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));

            // Operations for the elevator
            put(
                Operation.ElevatorBottomPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
            put(
                Operation.ElevatorCarryPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.ElevatorSwitchPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.ElevatorLowScalePosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_TOP_LEFT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.ElevatorHighScalePosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_TOP_RIGHT_BUTTON,
                    ButtonType.Click));
            put(
                Operation.ElevatorClimbPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
            put(
                Operation.ElevatorTopPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
            put(
                Operation.ElevatorMoveUp,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_RIGHT_BUTTON,
                    ButtonType.Simple));
            put(
                Operation.ElevatorMoveDown,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_RIGHT_BUTTON,
                    ButtonType.Simple));
            put(
                Operation.ElevatorIntake,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.NONE, // JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
                    ButtonType.Simple));
            put(
                Operation.ElevatorIntakeCorrection,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ElevatorOuttake,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_LEFT_BUTTON,
                    ButtonType.Simple));
            put(
                Operation.ElevatorIntakeArmsUp,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    180, // POV down
                    ButtonType.Click));
            put(
                Operation.ElevatorIntakeArmsDown,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    0, // POV up
                    ButtonType.Click));
            put(
                Operation.ElevatorIntakeFingersIn,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    90, // POV right
                    ButtonType.Simple));

            // Operations for the climber
            put(
                Operation.ClimberRelease,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    270, // POV left
                    ButtonType.Click));
            put(
                Operation.ClimberEnableWinch,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    270, // POV left
                    ButtonType.Click));
            put(
                Operation.ClimberDisableWinch,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
            put(
                Operation.ClimberWinch,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.Throttle,
                    ElectronicsConstants.INVERT_THROTTLE_AXIS,
                    TuningConstants.CLIMBER_WINCH_DEAD_ZONE));
        }
    };

    @SuppressWarnings("serial")
    public static Map<MacroOperation, MacroOperationDescription> MacroSchema = new HashMap<MacroOperation, MacroOperationDescription>()
    {
        {
            // Brake mode macro
            put(
                MacroOperation.PIDBrake,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_THUMB_BUTTON,
                    ButtonType.Simple,
                    () -> new PIDBrakeTask(),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));

            // Centering macro
            put(
                MacroOperation.VisionCenter,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle,
                    () -> new VisionCenteringTask(),
                    new Operation[]
                    {
                        Operation.EnableVision,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                    }));
            put(
                MacroOperation.VisionCenterAndAdvance,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(),
                    new Operation[]
                    {
                        Operation.EnableVision,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                    }));

            put(
                MacroOperation.IntakeAndCorrection,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
                    ButtonType.Simple,
                    () -> new IntakeAndCorrectionTask(),
                    new Operation[]
                    {
                        Operation.ElevatorIntake,
                        Operation.ElevatorIntakeCorrection,
                        Operation.ElevatorOuttake,
                        Operation.ElevatorIntakeFingersIn,
                    }));
            put(
                MacroOperation.ReIntake,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_RIGHT_BUTTON,
                    ButtonType.Simple,
                    () -> SequentialTask.Sequence(
                        new OuttakeTask(0.1),
                        new IntakeAndCorrectionTask()),
                    new Operation[]
                    {
                        Operation.ElevatorIntake,
                        Operation.ElevatorIntakeCorrection,
                        Operation.ElevatorOuttake,
                        Operation.ElevatorIntakeFingersIn,
                    }));
            put(
                MacroOperation.HookClimber,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_RIGHT_BUTTON,
                    ButtonType.Toggle,
                    () -> SequentialTask.Sequence(
                        new ElevatorMovementTask(5.0, Operation.ElevatorClimbPosition),
                        new ReleaseServoTimedTask(2.5),
                        new ElevatorMovementTask(1.0, Operation.ElevatorCarryPosition),
                        new EnableWinchTimedTask(.2)),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorCarryPosition,
                        Operation.ElevatorSwitchPosition,
                        Operation.ElevatorLowScalePosition,
                        Operation.ElevatorHighScalePosition,
                        Operation.ElevatorClimbPosition,
                        Operation.ElevatorTopPosition,
                        Operation.ClimberEnableWinch,
                        Operation.ClimberDisableWinch,
                        Operation.ClimberRelease,
                    }));
        }
    };

    @Override
    public Map<Operation, OperationDescription> getOperationSchema()
    {
        return ButtonMap.OperationSchema;
    }

    @Override
    public Map<MacroOperation, MacroOperationDescription> getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
