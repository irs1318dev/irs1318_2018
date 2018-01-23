package org.usfirst.frc.team1318.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = false;
    public static final boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;

    //================================================== Autonomous ==============================================================

    public static final double DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA = 1.0;

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // Acceptable vision distance from tape in inches
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 30.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.08;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.08;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.3;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.3;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.015;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.005;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    //================================================== DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean DRIVETRAIN_USE_PID = true;

    // Velocity PID (right)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP = 0.55;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF = 0.1;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KS = 5600.0;

    // Velocity PID (left)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP = 0.55;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF = 0.1;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KS = 5600.0;

    // Position PID (right)
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP = 0.0008;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF = 0.0;

    // Position PID (left)
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP = 0.0008;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF = 0.0;

    // Drivetrain choices for one-stick drive
    public static final double DRIVETRAIN_K1 = 1.4;
    public static final double DRIVETRAIN_K2 = 0.5;

    // Drivetrain deadzone/max power levels
    public static final double DRIVETRAIN_X_DEAD_ZONE = .05;
    public static final double DRIVETRAIN_Y_DEAD_ZONE = .1;
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 1.0;// max power level (velocity)
    public static final double DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE / 60.0;
    public static final double DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE / 60.0;
    public static final double DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2;// max power level (positional, non-PID)

    public static final double DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL = 0.6;
    public static final double DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL = 1.0;

    public static final double DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION = 1.0; // account for turning weirdness (any degree offset in the angle)

    //================================================== Elevator ==============================================================

    // Position PID (inner)
    public static final double ELEVATOR_POSITION_PID_INNER_KP = 0.0;
    public static final double ELEVATOR_POSITION_PID_INNER_KI = 0.0;
    public static final double ELEVATOR_POSITION_PID_INNER_KD = 0.0;
    public static final double ELEVATOR_POSITION_PID_INNER_KF = 0.0;

    // Position PID (outer)
    public static final double ELEVATOR_POSITION_PID_OUTER_KP = 0.0;
    public static final double ELEVATOR_POSITION_PID_OUTER_KI = 0.0;
    public static final double ELEVATOR_POSITION_PID_OUTER_KD = 0.0;
    public static final double ELEVATOR_POSITION_PID_OUTER_KF = 0.0;

    public static final boolean ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN = true;
    public static final boolean ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN = true;

    public static final boolean ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN = true;
    public static final boolean ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN = true;

    // Elevator motor/sensor orientations
    public static final boolean ELEVATOR_INNER_INVERT_OUTPUT = true;
    public static final boolean ELEVATOR_INNER_INVERT_SENSOR = true;
    public static final boolean ELEVATOR_OUTER_INVERT_OUTPUT = false;
    public static final boolean ELEVATOR_OUTER_INVERT_SENSOR = false;

    public static final boolean ELEVATOR_LEFT_CARRIAGE_INTAKE_INVERT_OUTPUT = false;
    public static final boolean ELEVATOR_RIGHT_CARRIAGE_INTAKE_INVERT_OUTPUT = false;
    public static final boolean ELEVATOR_LEFT_OUTER_INTAKE_INVERT_OUTPUT = false;
    public static final boolean ELEVATOR_RIGHT_OUTER_INTAKE_INVERT_OUTPUT = false;

    public static final double ELEVATOR_THROUGH_BEAM_UNBLOCKED_VOLTAGE_THRESHOLD = 3.0;

    // Elevator positions (in inches)
    public static final double ELEVATOR_INNER_CARRY_POSITION = 0;
    public static final double ELEVATOR_OUTER_CARRY_POSITION = 2.0;
    public static final double ELEVATOR_INNER_SWITCH_POSITION = 0;
    public static final double ELEVATOR_OUTER_SWITCH_POSITION = 2.0;
    public static final double ELEVATOR_INNER_LOW_SCALE_POSITION = 0;
    public static final double ELEVATOR_OUTER_LOW_SCALE_POSITION = 2.0;
    public static final double ELEVATOR_INNER_HIGH_SCALE_POSITION = 0;
    public static final double ELEVATOR_OUTER_HIGH_SCALE_POSITION = 2.0;

    public static final double ELEVATOR_MAXIMUM_OUTER_INTAKE_USE_HEIGHT = 12.0;

    // Elevator velocities
    public static final double ELEVATOR_MOVE_VELOCITY = 2.0; // inches per second

    // Elevator intake powers
    public static final double ELEVATOR_LEFT_OUTER_INTAKE_POWER = 0.2;
    public static final double ELEVATOR_RIGHT_OUTER_INTAKE_POWER = 0.2;
    public static final double ELEVATOR_LEFT_CARRIAGE_INTAKE_POWER = 0.7;
    public static final double ELEVATOR_RIGHT_CARRIAGE_INTAKE_POWER = 0.5;

    // Elevator outtake powers
    public static final double ELEVATOR_LEFT_OUTER_OUTTAKE_POWER = -0.1;
    public static final double ELEVATOR_RIGHT_OUTER_OUTTAKE_POWER = -0.1;
    public static final double ELEVATOR_LEFT_CARRIAGE_OUTTAKE_POWER = -0.2;
    public static final double ELEVATOR_RIGHT_CARRIAGE_OUTTAKE_POWER = -0.2;
}
