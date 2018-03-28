package org.usfirst.frc.team1318.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_Y_AXIS = true;

    // change INVERT_THROTTLE_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_THROTTLE_AXIS = true;

    public static final int PCM_A_MODULE = 0;
    public static final int PCM_B_MODULE = 1;

    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    //================================================== Auto ==============================================================

    public static final int AUTO_DIP_SWITCH_A_DIGITAL_CHANNEL = 0;
    public static final int AUTO_DIP_SWITCH_B_DIGITAL_CHANNEL = 1;
    public static final int AUTO_DIP_SWITCH_C_DIGITAL_CHANNEL = 2;
    public static final int AUTO_DIP_SWITCH_D_DIGITAL_CHANNEL = 3;
    public static final int AUTO_DIP_SWITCH_E_DIGITAL_CHANNEL = 4;
    public static final int AUTO_DIP_SWITCH_F_DIGITAL_CHANNEL = 5;

    //================================================== Vision ==============================================================

    public static final int VISION_RING_LIGHT_PCM_CHANNEL = 5; // Module A

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_MOTOR_CAN_ID = 1;
    public static final int DRIVETRAIN_LEFT_FOLLOWER_CAN_ID = 2;
    public static final int DRIVETRAIN_RIGHT_MOTOR_CAN_ID = 3;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID = 4;

    //================================================== Elevator ==============================================================

    // CAN:
    public static final int ELEVATOR_INNER_MOTOR_CAN_ID = 5;
    public static final int ELEVATOR_OUTER_MOTOR_CAN_ID = 6;
    public static final int ELEVATOR_TOP_CARRIAGE_INTAKE_MOTOR_CAN_ID = 7;
    public static final int ELEVATOR_BOTTOM_CARRIAGE_INTAKE_MOTOR_CAN_ID = 8;
    public static final int ELEVATOR_LEFT_OUTER_INTAKE_MOTOR_CAN_ID = 9;
    public static final int ELEVATOR_RIGHT_OUTER_INTAKE_MOTOR_CAN_ID = 10;

    // Analog I/O:
    public static final int ELEVATOR_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL = 0;
    public static final int ELEVATOR_OUTER_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL = 1;

    // PCM:
    public static final int ELEVATOR_INTAKE_ARM_PCM_CHANNEL_A = 0; // Module A
    public static final int ELEVATOR_INTAKE_ARM_PCM_CHANNEL_B = 1; // Module A
    public static final int ELEVATOR_INTAKE_FINGER_PCM_CHANNEL_A = 6; // Module A
    public static final int ELEVATOR_INTAKE_FINGER_PCM_CHANNEL_B = 7; // Module A
    public static final int ELEVATOR_COLLECTED_INDICATOR_LIGHT_PCM_CHANNEL = 2; // Module A
    public static final int ELEVATOR_POSITION_REACHED_INDICATOR_LIGHT_PCM_CHANNEL = 3; // Module A

    //================================================== Climber ==============================================================

    public static final int CLIMBER_WINCH_MOTOR_CAN_ID = 15; // Victor SPX
    public static final int CLIMBER_WINCH_FOLLOWER_CAN_ID = 16; // Victor SPX
}
