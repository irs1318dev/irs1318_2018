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
    private final ITalonSRX leftCarriageIntakeMotor;
    private final ITalonSRX rightCarriageIntakeMotor;
    private final ITalonSRX leftOuterIntakeMotor;
    private final ITalonSRX rightOuterIntakeMotor;

    private final IAnalogInput innerThroughBeamSensor;
    //    private final IAnalogInput outerThroughBeamSensor;

    private final IDoubleSolenoid intakeExtender;

    private final ISolenoid collectedIndicatorLight;

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

    private double innerThroughBeamVoltage;
    private boolean isInnerThroughBeamBlocked;

    private double outerThroughBeamVoltage;
    private boolean isOuterThroughBeamBlocked;

    private double desiredInnerHeight;
    private double desiredOuterHeight;
    private boolean shouldHold;

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

        this.innerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_MOTOR_CAN_ID);
        this.innerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.innerElevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_INNER_INVERT_OUTPUT);
        this.innerElevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_INNER_INVERT_SENSOR);
        this.innerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.innerElevatorMotor.reset();
        this.innerElevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.innerElevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        this.innerElevatorMotor.setPIDF(
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KP,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KI,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KD,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KF,
            ElevatorMechanism.pidSlotId);
        this.innerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);
        this.innerElevatorMotor.setControlMode(TalonSRXControlMode.Position);

        this.outerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_OUTER_MOTOR_CAN_ID);
        this.outerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.outerElevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_OUTER_INVERT_OUTPUT);
        this.outerElevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_OUTER_INVERT_SENSOR);
        this.outerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.outerElevatorMotor.reset();
        this.outerElevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.outerElevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        this.outerElevatorMotor.setPIDF(
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KP,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KI,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KD,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KF,
            ElevatorMechanism.pidSlotId);
        this.outerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);
        this.outerElevatorMotor.setControlMode(TalonSRXControlMode.Position);

        this.leftCarriageIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_MOTOR_CAN_ID);
        this.leftCarriageIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.leftCarriageIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_INVERT_OUTPUT);
        this.leftCarriageIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.rightCarriageIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_MOTOR_CAN_ID);
        this.rightCarriageIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.rightCarriageIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_INVERT_OUTPUT);
        this.rightCarriageIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.leftOuterIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_OUTER_INTAKE_MOTOR_CAN_ID);
        this.leftOuterIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.leftOuterIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_INVERT_OUTPUT);
        this.leftOuterIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);
        this.rightOuterIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_OUTER_INTAKE_MOTOR_CAN_ID);
        this.rightOuterIntakeMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.rightOuterIntakeMotor.setInvertOutput(TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_INVERT_OUTPUT);
        this.rightOuterIntakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        this.intakeExtender = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            ElectronicsConstants.ELEVATOR_INTAKE_ARM_PCM_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_INTAKE_ARM_PCM_CHANNEL_B);

        this.innerThroughBeamSensor = provider.getAnalogInput(ElectronicsConstants.ELEVATOR_INNER_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL);
        //        this.outerThroughBeamSensor = provider.getAnalogInput(ElectronicsConstants.ELEVATOR_OUTER_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL);

        this.collectedIndicatorLight = provider.getSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            ElectronicsConstants.ELEVATOR_COLLECTED_INDICATOR_LIGHT_PCM_CHANNEL);

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

        this.innerThroughBeamVoltage = 0.0;
        this.isInnerThroughBeamBlocked = false;
        this.outerThroughBeamVoltage = 0.0;
        this.isOuterThroughBeamBlocked = false;

        this.desiredInnerHeight = 0.0;
        this.desiredOuterHeight = 0.0;
        this.shouldHold = false;
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
     * get the total height of both elevators 
     * @return a height in inches
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
    public boolean getInnerThroughBeamStatus()
    {
        return this.isInnerThroughBeamBlocked;
    }

    /**
     * get the status from the outer through beam sensor
     * @return a value indicating whether the outer through beam sensor is blocked 
     */
    public boolean getOuterThroughBeamStatus()
    {
        return this.isOuterThroughBeamBlocked;
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
        this.innerThroughBeamVoltage = this.innerThroughBeamSensor.getVoltage();
        this.isInnerThroughBeamBlocked = this.innerThroughBeamVoltage < TuningConstants.ELEVATOR_THROUGH_BEAM_UNBLOCKED_VOLTAGE_THRESHOLD;

        //        this.outerThroughBeamVoltage = this.outerThroughBeamSensor.getVoltage();
        //        this.isOuterThroughBeamBlocked = this.outerThroughBeamVoltage < TuningConstants.ELEVATOR_THROUGH_BEAM_UNBLOCKED_VOLTAGE_THRESHOLD;

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
        this.logger.logNumber(ElevatorMechanism.LogName, "onnerThroughBeamSensorVoltage", this.innerThroughBeamVoltage);
        this.logger.logBoolean(ElevatorMechanism.LogName, "innerThroughBeamBlocked", this.isInnerThroughBeamBlocked);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerThroughBeamSensorVoltage", this.outerThroughBeamVoltage);
        this.logger.logBoolean(ElevatorMechanism.LogName, "outerThroughBeamBlocked", this.isOuterThroughBeamBlocked);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        //        if (this.innerElevatorReverseLimitSwitchStatus)
        //        {
        //            this.innerElevatorMotor.reset();
        //        }

        //        if (this.outerElevatorReverseLimitSwitchStatus)
        //        {
        //            this.outerElevatorMotor.reset();
        //        }

        boolean shouldIntake = this.driver.getDigital(Operation.ElevatorIntake);
        boolean shouldIntakeCorrection = this.driver.getDigital(Operation.ElevatorIntakeCorrection);
        boolean shouldOuttake = this.driver.getDigital(Operation.ElevatorOuttake);
        if (this.isInnerThroughBeamBlocked && !shouldOuttake)
        {
            this.shouldHold = true;
        }
        else if (shouldOuttake)
        {
            this.shouldHold = false;
        }

        if (this.driver.getDigital(Operation.ElevatorBottomPosition) || shouldIntake || shouldIntakeCorrection)
        {
            this.desiredInnerHeight = 0;
            this.desiredOuterHeight = 0;
        }
        else if (this.driver.getDigital(Operation.ElevatorCarryPosition))
        {
            this.desiredInnerHeight = TuningConstants.ELEVATOR_INNER_CARRY_POSITION;
            this.desiredOuterHeight = TuningConstants.ELEVATOR_OUTER_CARRY_POSITION;
        }
        else if (this.driver.getDigital(Operation.ElevatorSwitchPosition))
        {
            this.desiredInnerHeight = TuningConstants.ELEVATOR_INNER_SWITCH_POSITION;
            this.desiredOuterHeight = TuningConstants.ELEVATOR_OUTER_SWITCH_POSITION;
        }
        else if (this.driver.getDigital(Operation.ElevatorLowScalePosition))
        {
            this.desiredInnerHeight = TuningConstants.ELEVATOR_INNER_LOW_SCALE_POSITION;
            this.desiredOuterHeight = TuningConstants.ELEVATOR_OUTER_LOW_SCALE_POSITION;
        }
        else if (this.driver.getDigital(Operation.ElevatorHighScalePosition))
        {
            this.desiredInnerHeight = TuningConstants.ELEVATOR_INNER_HIGH_SCALE_POSITION;
            this.desiredOuterHeight = TuningConstants.ELEVATOR_OUTER_HIGH_SCALE_POSITION;
        }
        else if (this.driver.getDigital(Operation.ElevatorClimbPosition))
        {
            this.desiredInnerHeight = TuningConstants.ELEVATOR_INNER_CLIMB_POSITION;
            this.desiredOuterHeight = TuningConstants.ELEVATOR_OUTER_CLIMB_POSITION;
        }
        else if (this.driver.getDigital(Operation.ElevatorTopPosition))
        {
            this.desiredInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT;
            this.desiredOuterHeight = HardwareConstants.ELEVATOR_OUTER_MAX_HEIGHT;
        }
        else if (this.desiredInnerHeight + this.desiredOuterHeight == 0.0)
        {
            this.desiredInnerHeight = TuningConstants.ELEVATOR_INNER_CARRY_POSITION;
            this.desiredOuterHeight = TuningConstants.ELEVATOR_OUTER_CARRY_POSITION;
        }

        if (this.driver.getDigital(Operation.ElevatorMoveUp))
        {
            double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
            double remainingInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT - this.desiredInnerHeight;
            if (deltaPosition > remainingInnerHeight)
            {
                this.desiredInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT;
                this.desiredOuterHeight += (deltaPosition - remainingInnerHeight);
            }
            else
            {
                this.desiredInnerHeight += deltaPosition;
            }
        }
        else if (this.driver.getDigital(Operation.ElevatorMoveDown))
        {
            double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
            double remainingOuterHeight = this.desiredOuterHeight;
            if (deltaPosition > remainingOuterHeight)
            {
                this.desiredOuterHeight = 0.0;
                this.desiredInnerHeight -= (deltaPosition - remainingOuterHeight);
            }
            else
            {
                this.desiredOuterHeight -= deltaPosition;
            }
        }

        // Ensure that our desired inner and outer heights are within the permitted ranges:
        this.desiredInnerHeight = Helpers.EnforceRange(this.desiredInnerHeight, 0.0, HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT);
        this.desiredOuterHeight = Helpers.EnforceRange(this.desiredOuterHeight, 0.0, HardwareConstants.ELEVATOR_OUTER_MAX_HEIGHT);

        this.logger.logNumber(ElevatorMechanism.LogName, "desiredInnerHeight", this.desiredInnerHeight);
        this.logger.logNumber(ElevatorMechanism.LogName, "desiredOuterHeight", this.desiredOuterHeight);

        this.innerElevatorMotor.set(this.desiredInnerHeight / HardwareConstants.ELEVATOR_INNER_PULSE_DISTANCE);
        this.outerElevatorMotor.set(this.desiredOuterHeight / HardwareConstants.ELEVATOR_INNER_PULSE_DISTANCE);

        double leftOuterIntakePower = 0.0;
        double rightOuterIntakePower = 0.0;
        double leftCarriageIntakePower = 0.0;
        double rightCarriageIntakePower = 0.0;
        if (shouldIntake)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_POWER;
            leftCarriageIntakePower = TuningConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_POWER;
            rightCarriageIntakePower = TuningConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_POWER;
        }
        else if (shouldIntakeCorrection)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_CORRECTION_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_CORRECTION_POWER;
            leftCarriageIntakePower = TuningConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_CORRECTION_POWER;
            rightCarriageIntakePower = TuningConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_CORRECTION_POWER;
        }
        else if (shouldOuttake)
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_OUTTAKE_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_OUTTAKE_POWER;
            leftCarriageIntakePower = TuningConstants.ELEVATOR_LEFT_CARRIAGE_OUTTAKE_POWER;
            rightCarriageIntakePower = TuningConstants.ELEVATOR_RIGHT_CARRIAGE_OUTTAKE_POWER;
        }
        else if (this.shouldHold)
        {
            leftCarriageIntakePower = TuningConstants.ELEVATOR_LEFT_CARRIAGE_HOLD_POWER;
            rightCarriageIntakePower = TuningConstants.ELEVATOR_RIGHT_CARRIAGE_HOLD_POWER;
        }

        // Use outer intakes only if carriage is below a certain height
        if (this.innerElevatorHeight + this.outerElevatorHeight <= TuningConstants.ELEVATOR_MAXIMUM_OUTER_INTAKE_USE_HEIGHT)
        {
            this.leftOuterIntakeMotor.set(leftOuterIntakePower);
            this.rightOuterIntakeMotor.set(rightOuterIntakePower);
        }
        else
        {
            this.leftOuterIntakeMotor.set(0);
            this.rightOuterIntakeMotor.set(0);
        }

        this.leftCarriageIntakeMotor.set(leftCarriageIntakePower);
        this.rightCarriageIntakeMotor.set(rightCarriageIntakePower);

        this.collectedIndicatorLight.set(this.isInnerThroughBeamBlocked);

        if (this.driver.getDigital(Operation.ElevatorIntakeArmUp))
        {
            this.intakeExtender.set(DoubleSolenoidValue.kReverse);
        }
        else if (this.driver.getDigital(Operation.ElevatorIntakeArmDown))
        {
            this.intakeExtender.set(DoubleSolenoidValue.kForward);
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

        this.leftCarriageIntakeMotor.stop();
        this.rightCarriageIntakeMotor.stop();
        this.leftOuterIntakeMotor.stop();
        this.rightOuterIntakeMotor.stop();

        this.intakeExtender.set(DoubleSolenoidValue.kOff);
        this.collectedIndicatorLight.set(false);

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

        this.desiredInnerHeight = 0.0;
        this.desiredOuterHeight = 0.0;

        this.isInnerThroughBeamBlocked = false;
        this.isOuterThroughBeamBlocked = false;
    }
}
