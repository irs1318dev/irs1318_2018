package org.usfirst.frc.team1318.robot.elevator;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXLimitSwitchStatus;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXNeutralMode;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;

/**
 * Elevator mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 */
@Singleton
public class ElevatorMechanism implements IMechanism
{
    private static final String LogName = "el";
    private static final int pidSlotId = 0;

    private final IDashboardLogger logger;

    private final ITalonSRX innerElevatorMotor;
    private final ITalonSRX outerElevatorMotor;
    private final ITalonSRX leftCarriageIntakeMotor;
    private final ITalonSRX rightCarriageIntakeMotor;
    private final ITalonSRX leftOuterIntakeMotor;
    private final ITalonSRX rightOuterIntakeMotor;

    private Driver driver;

    private double innerElevatorVelocity;
    private double innerElevatorError;
    private int innerElevatorPosition;
    private boolean innerElevatorForwardLimitSwitchStatus;
    private boolean innerElevatorReverseLimitSwitchStatus;
    private double outerElevatorVelocity;
    private double outerElevatorError;
    private int outerElevatorPosition;
    private boolean outerElevatorForwardLimitSwitchStatus;
    private boolean outerElevatorReverseLimitSwitchStatus;

    /**
     * Initializes a new DriveTrainMechanism
     * @param logger to use
     * @param provider for obtaining electronics objects
     * @param timer to use
     */
    @Inject
    public ElevatorMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider)
    {
        this.logger = logger;

        this.innerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_MOTOR_CHANNEL);
        this.innerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.innerElevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_INNER_INVERT_OUTPUT);
        this.innerElevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_INNER_INVERT_SENSOR);
        this.innerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.innerElevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.innerElevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        this.outerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_OUTER_MOTOR_CHANNEL);
        this.outerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.outerElevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_OUTER_INVERT_OUTPUT);
        this.outerElevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_OUTER_INVERT_SENSOR);
        this.outerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.outerElevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.outerElevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        this.leftCarriageIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_MOTOR_CHANNEL);
        this.leftCarriageIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.leftCarriageIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_INVERT_OUTPUT);
        this.rightCarriageIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_MOTOR_CHANNEL);
        this.rightCarriageIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.rightCarriageIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_INVERT_OUTPUT);
        this.leftOuterIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_OUTER_INTAKE_MOTOR_CHANNEL);
        this.leftOuterIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.leftOuterIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_INVERT_OUTPUT);
        this.rightOuterIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_OUTER_INTAKE_MOTOR_CHANNEL);
        this.rightOuterIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.rightOuterIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_INVERT_OUTPUT);

        this.innerElevatorVelocity = 0.0;
        this.innerElevatorError = 0.0;
        this.innerElevatorPosition = 0;
        this.innerElevatorForwardLimitSwitchStatus = false;
        this.innerElevatorReverseLimitSwitchStatus = false;
        this.outerElevatorVelocity = 0.0;
        this.outerElevatorError = 0.0;
        this.outerElevatorPosition = 0;
        this.outerElevatorForwardLimitSwitchStatus = false;
        this.outerElevatorReverseLimitSwitchStatus = false;

        this.setControlMode();
    }

    /**
     * get the velocity from the inner elevator encoder
     * @return a value indicating the velocity
     */
    public double getInnerVelocity()
    {
        return this.innerElevatorVelocity;
    }

    /**
     * get the velocity from the outer elevator encoder
     * @return a value indicating the velocity
     */
    public double getOuterVelocity()
    {
        return this.outerElevatorVelocity;
    }

    /**
     * get the distance from the inner elevator encoder
     * @return a value indicating the distance
     */
    public double getInnerError()
    {
        return this.innerElevatorError;
    }

    /**
     * get the distance from the outer elevator encoder
     * @return a value indicating the distance
     */
    public double getOuterError()
    {
        return this.outerElevatorError;
    }

    /**
     * get the ticks from the inner elevator encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getInnerPosition()
    {
        return this.innerElevatorPosition;
    }

    /**
     * get the ticks from the outer elevator encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getOuterPosition()
    {
        return this.outerElevatorPosition;
    }

    /**
     * get the status from the inner elevator limit switch
     * @return a value indicating whether the limit switch is pressed
     */
    public boolean getInnerForwardLimitSwitchStatus()
    {
        return this.innerElevatorForwardLimitSwitchStatus;
    }

    /**
     * get the status from the outer elevator limit switch
     * @return a value indicating whether the limit switch is pressed
     */
    public boolean getOuterForwardLimitSwitchStatus()
    {
        return this.outerElevatorForwardLimitSwitchStatus;
    }

    /**
     * get the status from the inner elevator limit switch
     * @return a value indicating whether the limit switch is pressed
     */
    public boolean getInnerReverseLimitSwitchStatus()
    {
        return this.innerElevatorReverseLimitSwitchStatus;
    }

    /**
     * get the status from the outer elevator limit switch
     * @return a value indicating whether the limit switch is pressed
     */
    public boolean getOuterReverseLimitSwitchStatus()
    {
        return this.outerElevatorReverseLimitSwitchStatus;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.innerElevatorVelocity = this.innerElevatorMotor.getVelocity();
        this.innerElevatorError = this.innerElevatorMotor.getError();
        this.innerElevatorPosition = this.innerElevatorMotor.getPosition();

        TalonSRXLimitSwitchStatus innerLimitSwitchStatus = this.innerElevatorMotor.getLimitSwitchStatus();
        this.innerElevatorForwardLimitSwitchStatus = innerLimitSwitchStatus.isForwardClosed;
        this.innerElevatorReverseLimitSwitchStatus = innerLimitSwitchStatus.isReverseClosed;

        this.outerElevatorVelocity = this.outerElevatorMotor.getVelocity();
        this.outerElevatorError = this.outerElevatorMotor.getError();
        this.outerElevatorPosition = this.outerElevatorMotor.getPosition();

        TalonSRXLimitSwitchStatus outerLimitSwitchStatus = this.outerElevatorMotor.getLimitSwitchStatus();
        this.outerElevatorForwardLimitSwitchStatus = outerLimitSwitchStatus.isForwardClosed;
        this.outerElevatorReverseLimitSwitchStatus = outerLimitSwitchStatus.isReverseClosed;

        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorVelocity", this.innerElevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorError", this.innerElevatorError);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorPosition", this.innerElevatorPosition);
        this.logger.logBoolean(ElevatorMechanism.LogName, "innerElevatorReverseLimitSwitch", this.innerElevatorReverseLimitSwitchStatus);
        this.logger.logBoolean(ElevatorMechanism.LogName, "innerElevatorForwardLimitSwitch", this.innerElevatorForwardLimitSwitchStatus);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorVelocity", this.outerElevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorError", this.outerElevatorError);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorPosition", this.outerElevatorPosition);
        this.logger.logBoolean(ElevatorMechanism.LogName, "outerElevatorReverseLimitSwitch", this.outerElevatorReverseLimitSwitchStatus);
        this.logger.logBoolean(ElevatorMechanism.LogName, "outerElevatorForwardLimitSwitch", this.outerElevatorForwardLimitSwitchStatus);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        // apply the setpoints to the motors
        this.innerElevatorMotor.set(0);
        this.outerElevatorMotor.set(0);
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.innerElevatorMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.outerElevatorMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        this.innerElevatorMotor.stop();
        this.outerElevatorMotor.stop();

        this.innerElevatorMotor.reset();
        this.outerElevatorMotor.reset();

        this.innerElevatorVelocity = 0.0;
        this.innerElevatorError = 0.0;
        this.innerElevatorPosition = 0;
        this.outerElevatorVelocity = 0.0;
        this.outerElevatorError = 0.0;
        this.outerElevatorPosition = 0;
    }

    /**
     * set control mode based on our current settings
     */
    private void setControlMode()
    {
        this.innerElevatorMotor.setPIDF(
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KP,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KI,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KD,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KF,
            ElevatorMechanism.pidSlotId);
        this.outerElevatorMotor.setPIDF(
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KP,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KI,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KD,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KF,
            ElevatorMechanism.pidSlotId);

        this.innerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);
        this.outerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);

        TalonSRXControlMode mode = TalonSRXControlMode.Position;
        this.innerElevatorMotor.setControlMode(mode);
        this.outerElevatorMotor.setControlMode(mode);
    }
}
