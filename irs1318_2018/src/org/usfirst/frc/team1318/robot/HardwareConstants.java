package org.usfirst.frc.team1318.robot;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    //================================================== DriveTrain ==============================================================
    // Note: Pulse Distance is the distance moved per tick

    public static final double DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double DRIVETRAIN_LEFT_WHEEL_DIAMETER = 5.0; // (in inches)
    public static final double DRIVETRAIN_LEFT_PULSE_DISTANCE = Math.PI
        * HardwareConstants.DRIVETRAIN_LEFT_WHEEL_DIAMETER / HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION;

    public static final double DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double DRIVETRAIN_RIGHT_WHEEL_DIAMETER = 5.0; // (in inches)
    public static final double DRIVETRAIN_RIGHT_PULSE_DISTANCE = Math.PI
        * HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_DIAMETER / HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION;

    // measure from outside of wheel:
    public static final double DRIVETRAIN_WHEEL_SEPARATION_DISTANCE = 23.25; // (in inches)

    // DriveTrain motor/sensor orientations
    public static final boolean DRIVETRAIN_LEFT_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_LEFT_INVERT_SENSOR = true;
    public static final boolean DRIVETRAIN_RIGHT_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_INVERT_SENSOR = true;

    //================================================== Elevator ==============================================================

    public static final double ELEVATOR_INNER_MAX_HEIGHT = 36.5; // max height of the elevator
    public static final double ELEVATOR_OUTER_MAX_HEIGHT = 38.0; // max height of the elevator (carriage)

    public static final double ELEVATOR_INNER_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double ELEVATOR_INNER_ROTATION_TRAVEL = 4.00; // the amount of travel in the chain per rotation of the encoder (in inches)
    public static final double ELEVATOR_INNER_PULSE_DISTANCE = HardwareConstants.ELEVATOR_INNER_ROTATION_TRAVEL
        / HardwareConstants.ELEVATOR_INNER_ENCODER_PULSES_PER_REVOLUTION;

    public static final double ELEVATOR_OUTER_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
    public static final double ELEVATOR_OUTER_ROTATION_TRAVEL = 4.25; // the amount of travel in the chain per rotation of the encoder (in inches)
    public static final double ELEVATOR_OUTER_PULSE_DISTANCE = HardwareConstants.ELEVATOR_OUTER_ROTATION_TRAVEL
        / HardwareConstants.ELEVATOR_OUTER_ENCODER_PULSES_PER_REVOLUTION;
}
