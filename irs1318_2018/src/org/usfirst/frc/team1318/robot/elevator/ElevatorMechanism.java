package org.usfirst.frc.team1318.robot.elevator;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
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

    private static final int pidSlotId = -1;

    private final IDashboardLogger logger;
    private final ITimer timer;

    private final ITalonSRX innerElevatorMotor;
    private final ITalonSRX outerElevatorMotor;
    private final ITalonSRX innerLeftElevatorIntakeMotor;
    private final ITalonSRX innerRightElevatorIntakeMotor;

    private Driver driver;

    private boolean usePID;
    private boolean usePositionalMode;

    private PIDHandler innerCarriagePID;
    private PIDHandler outerCarriagePID;

    private double innerElevatorVelocity;
    private double innerElevatorError;
    private int innerElevatorPosition;
    private double outerElevatorVelocity;
    private double outerElevatorError;
    private int outerElevatorPosition;

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

        this.innerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_MOTOR_CHANNEL);
        this.innerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Coast);
        this.innerElevatorMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_LEFT_INVERT_OUTPUT);
        this.innerElevatorMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR);
        this.innerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        //this.innerCarriageMotor.setPIDF();

        this.outerElevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_OUTER_MOTOR_CHANNEL);
        this.outerElevatorMotor.setNeutralMode(TalonSRXNeutralMode.Coast);
        this.outerElevatorMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_OUTPUT);
        this.outerElevatorMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR);
        this.outerElevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        //this.outerCarriageMotor.setPIDF();

        this.innerLeftElevatorIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_LEFT_INTAKE_MOTOR_CHANNEL);
        this.innerRightElevatorIntakeMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_RIGHT_INTAKE_MOTOR_CHANNEL);

        this.innerCarriagePID = null;
        this.outerCarriagePID = null;

        this.usePositionalMode = true;

        this.innerElevatorVelocity = 0.0;
        this.innerElevatorError = 0.0;
        this.innerElevatorPosition = 0;
        this.outerElevatorVelocity = 0.0;
        this.outerElevatorError = 0.0;
        this.outerElevatorPosition = 0;
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getInnerVelocity()
    {
        return this.innerElevatorVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getOuterVelocity()
    {
        return this.outerElevatorVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getInnerError()
    {
        return this.innerElevatorError;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getOuterError()
    {
        return this.outerElevatorError;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getInnerPosition()
    {
        return this.innerElevatorPosition;
    }

    /**
     * get the ticks from the left encoder
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

        // switch to default velocity PID mode whenever we switch drivers (defense-in-depth)
        if (!this.usePID || this.usePositionalMode)
        {
            this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
            this.usePositionalMode = true;
        }

        this.setControlMode();
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

        // apply the power settings to the motors
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
     * create a PIDHandler based on our current settings
     */
    private void setControlMode()
    {
        TalonSRXControlMode mode = TalonSRXControlMode.PercentOutput;
        if (this.usePID)
        {
            if (this.usePositionalMode)
            {
                this.innerCarriagePID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KP,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KI,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KD,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    this.timer);
                this.outerCarriagePID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KP,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KI,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KD,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    this.timer);
            }
            else
            {
                this.innerCarriagePID = null;
                this.outerCarriagePID = null;
            }

            mode = TalonSRXControlMode.Velocity;
            this.innerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);
            this.outerElevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);
        }

        this.innerElevatorMotor.setControlMode(mode);
        this.outerElevatorMotor.setControlMode(mode);
    }

}
