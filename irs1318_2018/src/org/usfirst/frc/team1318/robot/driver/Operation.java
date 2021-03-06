package org.usfirst.frc.team1318.robot.driver;

public enum Operation
{
    // Vision operations:
    EnableVision,

    // DriveTrain operations:
    DriveTrainEnablePID,
    DriveTrainDisablePID,
    DriveTrainMoveForward,
    DriveTrainTurn,
    DriveTrainSimpleMode,
    DriveTrainUseBrakeMode,
    DriveTrainUsePositionalMode,
    DriveTrainLeftPosition,
    DriveTrainRightPosition,
    DriveTrainSwapFrontOrientation,

    // Elevator operations
    ElevatorBottomPosition,
    ElevatorCarryPosition,
    ElevatorSwitchPosition,
    ElevatorLowScalePosition,
    ElevatorHighScalePosition,
    ElevatorClimbPosition,
    ElevatorTopPosition,
    ElevatorMoveUp,
    ElevatorMoveDown,
    ElevatorForceUp,
    ElevatorForceDown,
    ElevatorIntake,
    ElevatorIntakeCorrection,
    ElevatorStrongOuttake,
    ElevatorWeakOuttake,
    ElevatorIntakeArmsUp,
    ElevatorIntakeArmsDown,
    ElevatorIntakeFingersIn,

    // Climber operations
    ClimberWinch, 
    ClimberEnableWinch,
    ClimberDisableWinch, 
}
