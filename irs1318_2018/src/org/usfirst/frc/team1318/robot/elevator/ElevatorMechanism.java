package org.usfirst.frc.team1318.robot.elevator;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.DoubleSolenoidValue;
import org.usfirst.frc.team1318.robot.common.wpilib.IDoubleSolenoid;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
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

    private final IDashboardLogger logger;
    private final ITimer timer;

    private final ITalonSRX innerElevatorMotor;
    private final ITalonSRX outerElevatorMotor;
    private final ITalonSRX leftCarriageIntakeMotor;
    private final ITalonSRX rightCarriageIntakeMotor;
    private final ITalonSRX leftOuterIntakeMotor;
    private final ITalonSRX rightOuterIntakeMotor;

    private final IDoubleSolenoid intakeExtender;

    private Driver driver;

    private double innerElevatorVelocity;
    private double innerElevatorError;
    private int innerElevatorPosition;
    private double outerElevatorVelocity;
    private double outerElevatorError;
    private int outerElevatorPosition;

    private double desiredInnerHeight;
    private double desiredOuterHeight;

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

        this.desiredInnerHeight = 0;
        this.desiredOuterHeight = 0;

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
        this.outerElevatorVelocity = 0.0;
        this.outerElevatorError = 0.0;
        this.outerElevatorPosition = 0;

        this.intakeExtender = provider.getDoubleSolenoid(ElectronicsConstants.ELEVATOR_INTAKE_ARM_CHANNEL_A,
            ElectronicsConstants.ELEVATOR_INTAKE_ARM_CHANNEL_B);

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
        this.outerElevatorVelocity = this.outerElevatorMotor.getVelocity();
        this.outerElevatorError = this.outerElevatorMotor.getError();
        this.outerElevatorPosition = this.outerElevatorMotor.getPosition();

        this.logger.logNumber(ElevatorMechanism.LogName, "innerCarriageVelocity", this.innerElevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerCarriageError", this.innerElevatorError);
        this.logger.logNumber(ElevatorMechanism.LogName, "innerCarriagePosition", this.innerElevatorPosition);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerCarriageVelocity", this.outerElevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerCarriageError", this.outerElevatorError);
        this.logger.logNumber(ElevatorMechanism.LogName, "outerCarriagePosition", this.outerElevatorPosition);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        if (this.driver.getDigital(Operation.ElevatorCarryPosition))
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

        if (this.driver.getDigital(Operation.ElevatorMoveUp))
        {
            double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
            double remainingInnerHeight = HardwareConstants.ELEVATOR_INNER_MAX_HEIGHT - desiredInnerHeight;

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
            double deltaPosition = -deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
            double remainingOuterHeight = desiredOuterHeight;

            if (deltaPosition < desiredOuterHeight)
            {
                this.desiredOuterHeight = 0;
                this.desiredInnerHeight += (deltaPosition - remainingOuterHeight);
            }
            else
            {
                this.desiredOuterHeight += deltaPosition;
            }
        }

        double leftOuterIntakePower = 0;
        double rightOuterIntakePower = 0;
        double leftCarriageIntakePower = 0;
        double rightCarriageIntakePower = 0;

        if (this.driver.getDigital(Operation.ElevatorIntake))
        {
            leftOuterIntakePower = TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_POWER;
            rightOuterIntakePower = TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_POWER;
            leftCarriageIntakePower = TuningConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_POWER;
            rightCarriageIntakePower = TuningConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_POWER;
        }
        else if (this.driver.getDigital(Operation.ElevatorOuttake))
        {
            leftOuterIntakePower = -TuningConstants.ELEVATOR_LEFT_OUTER_INTAKE_POWER;
            rightOuterIntakePower = -TuningConstants.ELEVATOR_RIGHT_OUTER_INTAKE_POWER;
            leftCarriageIntakePower = -TuningConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_POWER;
            rightCarriageIntakePower = -TuningConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_POWER;
        }

        if (this.driver.getDigital(Operation.ElevatorIntakeArmUp))
        {
            this.intakeExtender.set(DoubleSolenoidValue.kReverse);
        }
        else if (this.driver.getDigital(Operation.ElevatorIntakeArmDown))
        {
            this.intakeExtender.set(DoubleSolenoidValue.kForward);
        }

        lastUpdateTime = currentTime;
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

        this.intakeExtender.set(DoubleSolenoidValue.kOff);
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
