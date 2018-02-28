package org.usfirst.frc.team1318.robot.climber;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.IServo;
import org.usfirst.frc.team1318.robot.common.wpilib.IVictorSPX;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
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
public class ClimberMechanism implements IMechanism
{
    private static final String LogName = "c";

    private final IDashboardLogger logger;

    private final IServo releaser;
    private final IVictorSPX winch;

    private Driver driver;

    private boolean winchEnabled;
    private boolean hookReleased;

    /**
     * Initializes a new DriveTrainMechanism
     * @param logger to use
     * @param provider for obtaining electronics objects
     */
    @Inject
    public ClimberMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider)
    {
        this.logger = logger;

        this.winch = provider.getVictorSPX(ElectronicsConstants.CLIMBER_WINCH_MOTOR_CAN_ID);
        this.winch.setInvertOutput(true);

        if (TuningConstants.COMPETITION_ROBOT)
        {
            IVictorSPX winchFollower = provider.getVictorSPX(ElectronicsConstants.CLIMBER_WINCH_FOLLOWER_CAN_ID);
            winchFollower.setInvertOutput(true);
            winchFollower.setControlMode(TalonSRXControlMode.Follower);
            winchFollower.set(ElectronicsConstants.CLIMBER_WINCH_MOTOR_CAN_ID);
        }

        this.releaser = provider.getServo(ElectronicsConstants.CLIMBER_RELEASER_SERVO_PWM_CHANNEL);

        this.winchEnabled = false;
        this.hookReleased = false;
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
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (this.driver.getDigital(Operation.ClimberEnableWinch))
        {
            this.winchEnabled = true;
        }
        else if (this.driver.getDigital(Operation.ClimberDisableWinch))
        {
            this.winchEnabled = false;
        }

        if (this.driver.getDigital(Operation.ClimberRelease))
        {
            this.hookReleased = true;
        }

        this.logger.logBoolean(ClimberMechanism.LogName, "winchEnabled", this.winchEnabled);
        this.logger.logBoolean(ClimberMechanism.LogName, "hookReleased", this.hookReleased);

        if (this.hookReleased)
        {
            this.releaser.set(1.0);
        }
        else
        {
            this.releaser.set(0.0);
        }

        if (this.winchEnabled)
        {
            double winchSpeed = driver.getAnalog(Operation.ClimberWinch);
            if (winchSpeed > 0)
            {
                this.winch.set(winchSpeed);
            }
            else
            {
                this.winch.set(0.0);
            }
        }
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.winch.set(0.0);
    }
}
