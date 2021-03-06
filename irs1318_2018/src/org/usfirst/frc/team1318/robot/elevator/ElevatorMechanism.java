package org.usfirst.frc.team1318.robot.elevator;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.Helpers;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.DoubleSolenoidValue;
import org.usfirst.frc.team1318.robot.common.wpilib.IAnalogInput;
import org.usfirst.frc.team1318.robot.common.wpilib.IDoubleSolenoid;
import org.usfirst.frc.team1318.robot.common.wpilib.ISolenoid;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXLimitSwitchStatus;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXNeutralMode;
import org.usfirst.frc.team1318.robot.driver.Operation;
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

    private final ITimer timer;

    private final IDashboardLogger logger;

    private final ITalonSRX innerElevatorMotor;
    private final ITalonSRX outerElevatorMotor;
    private final ITalonSRX topCarriageIntakeMotor;
    private final ITalonSRX bottomCarriageIntakeMotor;
    private final ITalonSRX leftOuterIntakeMotor;
    private final ITalonSRX rightOuterIntakeMotor;

    private final IAnalogInput throughBeamSensor;

    private final IDoubleSolenoid intakeArmExtender;
    private final IDoubleSolenoid intakeFingerExtender;

    private final ISolenoid collectedIndicatorLight;
    private final ISolenoid positionReachedIndicatorLight;

    private final TalonSRXControlMode pidControlMode;

    private Driver driver;

    private double innerElevatorVelocity;
    private double innerElevatorError;
    private int innerElevatorPosition;
    private double innerElevatorHeight;
    private boolean innerElevatorForwardLimitSwitchStatus;
    private boolean innerElevatorReverseLimitSwitchStatus;
    private double outerElevatorVelocity;
    private double outerElevatorError;
    private int outerElevatorPosition;
    private double outerElevatorHeight;
    private boolean outerElevatorForwardLimitSwitchStatus;
    private boolean outerElevatorReverseLimitSwitchStatus;

    private double throughBeamVoltage;
    private boolean isThroughBeamBlocked;

    private double desiredInnerHeight;
    private double desiredOuterHeight;
    private boolean shouldHold;
    private boolean isIntakeArmDown;

    private double lastUpdateTime;

    /**
     * Initializes a new DriveTrainMechanism
     * @param logger to use
     * @param provider for obtaining electronics objects
     * @param timer to use
     */
    @Inject
    public ElevatorMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider,
        ITimer timer)
    {
        this.logger = logger;
        this.timer = timer;

        // determine our PID control methodology from settings...
        this.pidControlMode = TuningConstants.ELEVATOR_USE_MOTION_MAGIC
            ? TalonSRXControlMode.MotionMagicPosition
            : TalonSRXControlMode.Position;

        this.innerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_MOTOR_CAN_ID);
        this.innerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.innerElevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_INNER_INVERT_OUTPUT);
        this.innerElevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_INNER_INVERT_SENSOR);
        this.innerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.innerElevatorMotor.setPosition(
            (int)(TuningConstants.ELEVATOR_INNER_CARRY_POSITION / HardwareConstants.ELEVATOR_INNER_PULSE_DISTANCE));
        this.innerElevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.innerElevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        if (TuningConstants.ELEVATOR_USE_MOTION_MAGIC)
        {
            this.innerElevatorMotor.setMotionMagicPIDF(
                TuningConstants.ELEVATOR_MM_POSITION_PID_INNER_KP,
                TuningConstants.ELEVATOR_MM_POSITION_PID_INNER_KI,
                TuningConstants.ELEVATOR_MM_POSITION_PID_INNER_KD,
                TuningConstants.ELEVATOR_MM_POSITION_PID_INNER_KF,
                TuningConstants.ELEVATOR_MM_POSITION_PID_INNER_CRUISE_VELOC,
                TuningConstants.ELEVATOR_MM_POSITION_PID_INNER_ACCEL,
                ElevatorMechanism.pidSlotId);
        }
        else
        {
            this.innerElevatorMotor.setPIDF(
                TuningConstants.ELEVATOR_POSITION_PID_INNER_KP,
                TuningConstants.ELEVATOR_POSITION_PID_INNER_KI,
                TuningConstants.ELEVATOR_POSITION_PID_INNER_KD,
                TuningConstants.ELEVATOR_POSITION_PID_INNER_KF,
                ElevatorMechanism.pidSlotId);
        }

        this.innerElevatorMotor.setControlMode(this.pidControlMode);
        this.innerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);

        this.outerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_OUTER_MOTOR_CAN_ID);
        this.outerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.outerElevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_OUTER_INVERT_OUTPUT);
        this.outerElevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_OUTER_INVERT_SENSOR);
        this.outerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.outerElevatorMotor.setPosition(
            (int)(TuningConstants.ELEVATOR_OUTER_CARRY_POSITION / HardwareConstants.ELEVATOR_OUTER_PULSE_DISTANCE));
        this.outerElevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.outerElevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        if (TuningConstants.ELEVATOR_USE_MOTION_MAGIC)
        {
            this.outerElevatorMotor.setMotionMagicPIDF(
                TuningConstants.ELEVATOR_MM_POSITION_PID_OUTER_KP,
                TuningConstants.ELEVATOR_MM_POSITION_PID_OUTER_KI,
                TuningConstants.ELEVATOR_MM_POSITION_PID_OUTER_KD,
                TuningConstants.ELEVATOR_MM_POSITION_PID_OUTER_KF,
                TuningConstants.ELEVATOR_MM_POSITION_PID_OUTER_CRUISE_VELOC,
                TuningConstants.ELEVATOR_MM_POSITION_PID_OUTER_ACCEL,
                ElevatorMechanism.pidSlotId);
        }
        else
        {
            this.outerElevatorMotor.setPIDF(
                TuningConstants.ELEVATOR_POSITION_PID_OUTER_KP,
                TuningConstants.ELEVATOR_POSITION_PID_OUTER_KI,
                TuningConstants.ELEVATOR_POSITION_PID_OUTER_KD,
                TuningConstants.ELEVATOR_POSITION_PID_OUTER_KF,
                ElevatorMechanism.pidSlotId);
        }

        this.outerElevatorMotor.setControlMode(this.pidControlMode);
        this.outerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);

        this.topCarriageIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_TOP_CARRIAGE_INTAKE_MOTOR_CAN_ID);
        this.topCarriageIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.topCarriageIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_TOP_CARRIAGE_INTAKE_INVERT_OUTPUT);
        this.topCarriageIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.bottomCarriageIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_BOTTOM_CARRIAGE_INTAKE_MOTOR_CAN_ID);
        this.bottomCarriageIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.bottomCarriageIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_BOTTOM_CARRIAGE_INTAKE_INVERT_OUTPUT);
        this.bottomCarriageIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.leftOuterIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_OUTER_INTAKE_MOTOR_CAN_ID);
        this.leftOuterIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.leftOuterIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_INVERT_OUTPUT);
        this.leftOuterIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.rightOuterIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_OUTER_INTAKE_MOTOR_CAN_ID);
        this.rightOuterIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.rightOuterIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_INVERT_OUTPUT);
        this.rightOuterIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        this.intakeArmExtender = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            ElectronicsConstants.ELEVATOR_INTAKE_ARM_PCM_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_INTAKE_ARM_PCM_CHANNEL_B);

        this.intakeFingerExtender = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            ElectronicsConstants.ELEVATOR_INTAKE_FINGER_PCM_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_INTAKE_FINGER_PCM_CHANNEL_B);

        this.throughBeamSensor = provider.getAnalogInput(ElectronicsConstants.ELEVATOR_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL);

        this.collectedIndicatorLight = provider.getSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            ElectronicsConstants.ELEVATOR_COLLECTED_INDICATOR_LIGHT_PCM_CHANNEL);

        this.positionReachedIndicatorLight = provider.getSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            ElectronicsConstants.ELEVATOR_POSITION_REACHED_INDICATOR_LIGHT_PCM_CHANNEL);

        this.innerElevatorVelocity = 0.0;
        this.innerElevatorError = 0.0;
        this.innerElevatorPosition = 0;
        this.innerElevatorHeight = 0.0;
        this.innerElevatorForwardLimitSwitchStatus = false;
        this.innerElevatorReverseLimitSwitchStatus = false;
        this.outerElevatorVelocity = 0.0;
        this.outerElevatorError = 0.0;
        this.outerElevatorPosition = 0;
        this.outerElevatorHeight = 0.0;
        this.outerElevatorForwardLimitSwitchStatus = false;
        this.outerElevatorReverseLimitSwitchStatus = false;

        this.throughBeamVoltage = 0.0;
        this.isThroughBeamBlocked = false;

        this.desiredInnerHeight = 0.0;
        this.desiredOuterHeight = 0.0;
        this.shouldHold = false;
        this.isIntakeArmDown = false;
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
     * get the total error of both elevators 
     * @return a height in encoder ticks
     */
    public double getTotalError()
    {
        return this.innerElevatorError + this.outerElevatorError;
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
     * get the status from the inner through beam sensor
     * @return a value indicating whether the inner through beam sensor is blocked 
     */
    public boolean getThroughBeamStatus()
    {
        return this.isThroughBeamBlocked;
    }

    /**
     * get a value showing if the arms are up or down
     * @return a value indicating whether the arms are up or down
     */
    public boolean getArmDownStatus()
    {
        return this.isIntakeArmDown;
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
        this.throughBeamVoltage = this.throughBeamSensor.getVoltage();
        this.isThroughBeamBlocked = this.throughBeamVoltage < TuningConstants.ELEVATOR_THROUGH_BEAM_UNBLOCKED_VOLTAGE_THRESHOLD;

        this.innerElevatorVelocity = this.innerElevatorMotor.getVelocity();
        this.innerElevatorError = this.innerElevatorMotor.getError();
        this.innerElevatorPosition = this.innerElevatorMotor.getPosition();
        this.innerElevatorHeight = this.innerElevatorPosition * HardwareConstants.ELEVATOR_INNER_PULSE_DISTANCE;

        TalonSRXLimitSwitchStatus innerLimitSwitchStatus = this.innerElevatorMotor.getLimitSwitchStatus();
        this.innerElevatorForwardLimitSwitchStatus = innerLimitSwitchStatus.isForwardClosed;
        this.innerElevatorReverseLimitSwitchStatus = innerLimitSwitchStatus.isReverseClosed;

        this.outerElevatorVelocity = this.outerElevatorMotor.getVelocity();
        this.outerElevatorError = this.outerElevatorMotor.getError();
        this.outerElevatorPosition = this.outerElevatorMotor.getPosition();
        this.outerElevatorHeight = this.outerElevatorPosition * HardwareConstants.ELEVATOR_OUTER_PULSE_DISTANCE;

        TalonSRXLimitSwitchStatus outerLimitSwitchStatus = this.outerElevatorMotor.getLimitSwitchStatus();
        this.outerElevatorForwardLimitSwitchStatus = outerLimitSwitchStatus.isForwardClosed;
        this.outerElevatorReverseLimitSwitchStatus = outerLimitSwitchStatus.isReverseClosed;

        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorVelocity", this.innerElevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorError", this.innerElevatorError);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorPosition", this.innerElevatorPosition);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerElevatorHeight", this.innerElevatorHeight);
        this.logger.logBoolean(ElevatorMechanism.LogName, "innerElevatorReverseLimitSwitch", this.innerElevatorReverseLimitSwitchStatus);
        this.logger.logBoolean(ElevatorMechanism.LogName, "innerElevatorForwardLimitSwitch", this.innerElevatorForwardLimitSwitchStatus);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorVelocity", this.outerElevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorError", this.outerElevatorError);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorPosition", this.outerElevatorPosition);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerElevatorHeight", this.outerElevatorHeight);
        this.logger.logBoolean(ElevatorMechanism.LogName, "outerElevatorReverseLimitSwitch", this.outerElevatorReverseLimitSwitchStatus);
        this.logger.logBoolean(ElevatorMechanism.LogName, "outerElevatorForwardLimitSwitch", this.outerElevatorForwardLimitSwitchStatus);
        this.logger.logNumber(ElevatorMechanism.LogName, "throughBeamSensorVoltage", this.throughBeamVoltage);
        this.logger.logBoolean(ElevatorMechanism.LogName, "throughBeamBlocked", this.isThroughBeamBlocked);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        double currentTotalHeight = this.innerElevatorHeight + this.outerElevatorHeight;

        boolean moveArmUp = this.driver.getDigital(Operation.ElevatorIntakeArmsUp);
        boolean moveArmDown = this.driver.getDigital(Operation.ElevatorIntakeArmsDown);
        boolean shouldIntake = this.driver.getDigital(Operation.ElevatorIntake);
        boolean shouldIntakeCorrection = this.driver.getDigital(Operation.ElevatorIntakeCorrection);
        boolean shouldStrongOuttake = this.driver.getDigital(Operation.ElevatorStrongOuttake);
        boolean shouldWeakOuttake = this.driver.getDigital(Operation.ElevatorWeakOuttake);
        if (shouldIntake || shouldIntakeCorrection)
        {
            this.shouldHold = true;
        }
        else if (shouldStrongOuttake || shouldWeakOuttake)
        {
            this.shouldHold = false;
        }

        boolean forceUp = this.driver.getDigital(Operation.ElevatorForceUp);
        boolean forceDown = this.driver.getDigital(Operation.ElevatorForceDown);
        if (forceUp || forceDown)
        {
            this.desiredInnerHeight = this.innerElevatorHeight;
            this.desiredOuterHeight = this.outerElevatorHeight;
            if (this.innerElevatorReverseLimitSwitchStatus || this.innerElevatorPosition < 0.0)
            {
                this.desiredInnerHeight = 0.0;
                this.innerElevatorMotor.reset();
            }

            if (this.outerElevatorReverseLimitSwitchStatus || this.outerElevatorPosition < 0.0)
            {
                this.desiredOuterHeight = 0.0;
                this.outerElevatorMotor.reset();
            }

            this.innerElevatorMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            this.outerElevatorMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            if (forceUp)
            {
                this.innerElevatorMotor.set(
                    this.innerElevatorForwardLimitSwitchStatus ? 0.0 : TuningConstants.ELEVATOR_DEBUG_UP_POWER_LEVEL);
                this.outerElevatorMotor.set(
                    this.outerElevatorForwardLimitSwitchStatus ? 0.0 : TuningConstants.ELEVATOR_DEBUG_UP_POWER_LEVEL);
            }
            else if (forceDown)
            {
                this.innerElevatorMotor.set(
                    this.innerElevatorReverseLimitSwitchStatus ? 0.0 : -TuningConstants.ELEVATOR_DEBUG_DOWN_POWER_LEVEL);
                this.outerElevatorMotor.set(
                    this.outerElevatorReverseLimitSwitchStatus ? 0.0 : -TuningConstants.ELEVATOR_DEBUG_DOWN_POWER_LEVEL);
            }
        }
        else
        {
            double newDesiredInnerHeight = this.desiredInnerHeight;
            double newDesiredOuterHeight = this.desiredOuterHeight;
            if (this.driver.getDigital(Operation.ElevatorBottomPosition)
                || shouldIntake || shouldIntakeCorrection
                || ((shouldWeakOuttake || shouldStrongOuttake)
                    && this.desiredInnerHeight == TuningConstants.ELEVATOR_INNER_CARRY_POSITION
                    && this.desiredOuterHeight == TuningConstants.ELEVATOR_OUTER_CARRY_POSITION))
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_BOTTOM_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_BOTTOM_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorCarryPosition))
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_CARRY_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_CARRY_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorSwitchPosition))
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_SWITCH_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_SWITCH_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorLowScalePosition))
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_LOW_SCALE_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_LOW_SCALE_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorHighScalePosition))
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_HIGH_SCALE_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_HIGH_SCALE_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorClimbPosition))
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_CLIMB_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_CLIMB_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorTopPosition))
            {
                newDesiredInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT;
                newDesiredOuterHeight = HardwareConstants.ELEVATOR_OUTER_MAX_HEIGHT;
            }
            else if ((this.desiredInnerHeight < TuningConstants.ELEVATOR_INNER_CARRY_POSITION
                || this.desiredOuterHeight < TuningConstants.ELEVATOR_OUTER_CARRY_POSITION)
                && !shouldStrongOuttake && !shouldWeakOuttake
                && !shouldIntake && !shouldIntakeCorrection)
            {
                newDesiredInnerHeight = TuningConstants.ELEVATOR_INNER_CARRY_POSITION;
                newDesiredOuterHeight = TuningConstants.ELEVATOR_OUTER_CARRY_POSITION;
            }

            if (this.driver.getDigital(Operation.ElevatorMoveUp))
            {
                double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
                double remainingInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT - newDesiredInnerHeight;
                if (deltaPosition > remainingInnerHeight)
                {
                    newDesiredInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT;
                    newDesiredOuterHeight += (deltaPosition - remainingInnerHeight);
                }
                else
                {
                    newDesiredInnerHeight += deltaPosition;
                }
            }
            else if (this.driver.getDigital(Operation.ElevatorMoveDown))
            {
                double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
                double remainingOuterHeight = newDesiredOuterHeight;
                if (deltaPosition > remainingOuterHeight)
                {
                    newDesiredOuterHeight = 0.0;
                    newDesiredInnerHeight -= (deltaPosition - remainingOuterHeight);
                }
                else
                {
                    newDesiredOuterHeight -= deltaPosition;
                }
            }

            this.desiredInnerHeight =
                Helpers.EnforceRange(
                    newDesiredInnerHeight,
                    TuningConstants.ELEVATOR_INNER_BOTTOM_POSITION,
                    HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT);
            this.desiredOuterHeight =
                Helpers.EnforceRange(
                    newDesiredOuterHeight,
                    TuningConstants.ELEVATOR_OUTER_BOTTOM_POSITION,
                    HardwareConstants.ELEVATOR_OUTER_MAX_HEIGHT);

            this.logger.logNumber(ElevatorMechanism.LogName, "desiredInnerHeight", this.desiredInnerHeight);
            this.logger.logNumber(ElevatorMechanism.LogName, "desiredOuterHeight", this.desiredOuterHeight);

            this.innerElevatorMotor.setControlMode(this.pidControlMode);
            this.outerElevatorMotor.setControlMode(this.pidControlMode);
            if (TuningConstants.ELEVATOR_USE_CLUTCH
                && Helpers.WithinDelta(this.outerElevatorHeight, this.desiredOuterHeight, TuningConstants.ELEVATOR_CLUTCH_POSITION_DELTA))
            {
                this.outerElevatorMotor.stop();
            }
            else
            {
                this.outerElevatorMotor.set(this.desiredOuterHeight / HardwareConstants.ELEVATOR_INNER_PULSE_DISTANCE);
            }

            this.innerElevatorMotor.set(this.desiredInnerHeight / HardwareConstants.ELEVATOR_INNER_PULSE_DISTANCE);
        }

        double leftOuterIntakePower = 0.0;
        double rightOuterIntakePower = 0.0;
        double topCarriageIntakePower = 0.0;
        double bottomCarriageIntakePower = 0.0;
        if (shouldIntake)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_POWER;
            topCarriageIntakePower = TuningConstants.ELEVATOR_TOP_CARRIAGE_INTAKE_POWER;
            bottomCarriageIntakePower = TuningConstants.ELEVATOR_BOTTOM_CARRIAGE_INTAKE_POWER;
        }
        else if (shouldIntakeCorrection)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_CORRECTION_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_CORRECTION_POWER;
            topCarriageIntakePower = TuningConstants.ELEVATOR_TOP_CARRIAGE_INTAKE_POWER;
            bottomCarriageIntakePower = TuningConstants.ELEVATOR_BOTTOM_CARRIAGE_INTAKE_POWER;
        }
        else if (shouldStrongOuttake)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_STRONG_OUTTAKE_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_STRONG_OUTTAKE_POWER;
            topCarriageIntakePower = TuningConstants.ELEVATOR_TOP_CARRIAGE_STRONG_OUTTAKE_POWER;
            bottomCarriageIntakePower = TuningConstants.ELEVATOR_BOTTOM_CARRIAGE_STRONG_OUTTAKE_POWER;
        }
        else if (shouldWeakOuttake)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_WEAK_OUTTAKE_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_WEAK_OUTTAKE_POWER;
            topCarriageIntakePower = TuningConstants.ELEVATOR_TOP_CARRIAGE_WEAK_OUTTAKE_POWER;
            bottomCarriageIntakePower = TuningConstants.ELEVATOR_BOTTOM_CARRIAGE_WEAK_OUTTAKE_POWER;
        }
        else if (this.shouldHold)
        {
            topCarriageIntakePower = TuningConstants.ELEVATOR_TOP_CARRIAGE_HOLD_POWER;
            bottomCarriageIntakePower = TuningConstants.ELEVATOR_BOTTOM_CARRIAGE_HOLD_POWER;
        }

        // Use outer intakes only if carriage is below a certain height and the intake arm is down
        if (this.isIntakeArmDown
            && currentTotalHeight <= TuningConstants.ELEVATOR_MAXIMUM_OUTER_INTAKE_USE_HEIGHT)
        {
            this.leftOuterIntakeMotor.set(leftOuterIntakePower);
            this.rightOuterIntakeMotor.set(rightOuterIntakePower);

            if (this.driver.getDigital(Operation.ElevatorIntakeFingersIn))
            {
                this.intakeFingerExtender.set(DoubleSolenoidValue.kReverse);
            }
            else
            {
                this.intakeFingerExtender.set(DoubleSolenoidValue.kForward);
            }
        }
        else
        {
            this.leftOuterIntakeMotor.set(0);
            this.rightOuterIntakeMotor.set(0);

            this.intakeFingerExtender.set(DoubleSolenoidValue.kForward);
        }

        this.topCarriageIntakeMotor.set(topCarriageIntakePower);
        this.bottomCarriageIntakeMotor.set(bottomCarriageIntakePower);

        this.collectedIndicatorLight.set(this.isThroughBeamBlocked);
        this.positionReachedIndicatorLight.set(
            this.isThroughBeamBlocked // Turn on all lights when power cube is collected
        /* Helpers.WithinDelta(
             currentTotalHeight,
             this.desiredInnerHeight + this.desiredOuterHeight,
             TuningConstants.ELEVATOR_POSITION_REACHED_DELTA)*/
        );

        if (moveArmDown)
        {
            this.isIntakeArmDown = true;
            this.intakeArmExtender.set(DoubleSolenoidValue.kForward);
        }
        else if (moveArmUp) //&& !isWithinRestrictedRange && !desiresWithinRestrictedRange)
        {
            this.isIntakeArmDown = false;
            this.intakeArmExtender.set(DoubleSolenoidValue.kReverse);
        }

        this.lastUpdateTime = currentTime;
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.innerElevatorMotor.stop();
        this.outerElevatorMotor.stop();

        this.topCarriageIntakeMotor.stop();
        this.bottomCarriageIntakeMotor.stop();
        this.leftOuterIntakeMotor.stop();
        this.rightOuterIntakeMotor.stop();

        this.intakeArmExtender.set(DoubleSolenoidValue.kOff);
        this.intakeFingerExtender.set(DoubleSolenoidValue.kOff);
        this.collectedIndicatorLight.set(false);
        this.positionReachedIndicatorLight.set(false);

        this.innerElevatorVelocity = 0.0;
        this.innerElevatorError = 0.0;
        this.innerElevatorPosition = 0;
        this.innerElevatorHeight = 0.0;
        this.innerElevatorForwardLimitSwitchStatus = false;
        this.innerElevatorReverseLimitSwitchStatus = false;
        this.outerElevatorVelocity = 0.0;
        this.outerElevatorError = 0.0;
        this.outerElevatorPosition = 0;
        this.outerElevatorHeight = 0.0;
        this.outerElevatorForwardLimitSwitchStatus = false;
        this.outerElevatorReverseLimitSwitchStatus = false;

        this.isThroughBeamBlocked = false;
    }
}
