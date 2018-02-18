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
    ElevatorIntake,
    ElevatorIntakeCorrection,
    ElevatorOuttake,
    ElevatorIntakeArmsUp,
    ElevatorIntakeArmsDown,
    ElevatorIntakeFingersOut,
    ElevatorIntakeFingersIn,
     
    // Climber operations
    ClimberRelease, 
    ClimberWinch, 
    ClimberEnableWinch,
    ClimberDisableWinch, 
    
}
