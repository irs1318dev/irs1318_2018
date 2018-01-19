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

    public static final int PCM_A_MODULE = 0;
    public static final int PCM_B_MODULE = 1;

    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    //================================================== Auto ==============================================================

    public static final int AUTO_DIP_SWITCH_A_CHANNEL = -1;
    public static final int AUTO_DIP_SWITCH_B_CHANNEL = -1;
    public static final int AUTO_DIP_SWITCH_C_CHANNEL = -1;
    public static final int AUTO_DIP_SWITCH_D_CHANNEL = -1;

    //================================================== Vision ==============================================================

    public static final int VISION_RING_LIGHT_CHANNEL = 2;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_MOTOR_CHANNEL = 1;
    public static final int DRIVETRAIN_LEFT_FOLLOWER_CHANNEL = 2;
    public static final int DRIVETRAIN_RIGHT_MOTOR_CHANNEL = 3;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER_CHANNEL = 4;

    //================================================== Elevator ==============================================================
    public static final int ELEVATOR_INNER_MOTOR_CHANNEL = -1;
    public static final int ELEVATOR_OUTER_MOTOR_CHANNEL = -1;
    public static final int ELEVATOR_TOP_LIMIT_CHANNEL = -1;
    public static final int ELEVATOR_BOTTOM_LIMIT_CHANNEL = -1;
}
