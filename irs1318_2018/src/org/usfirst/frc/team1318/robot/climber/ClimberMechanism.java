package org.usfirst.frc.team1318.robot.climber;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.DoubleSolenoidValue;
import org.usfirst.frc.team1318.robot.common.wpilib.IDoubleSolenoid;
import org.usfirst.frc.team1318.robot.common.wpilib.IMotor;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
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

    private final IMotor telescopingArm;
    private final IMotor winch;

    private final IDoubleSolenoid stower;
    private final IDoubleSolenoid leaner;

    private Driver driver;

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
        this.telescopingArm = provider.getTalon(ElectronicsConstants.CLIMBER_TELESCOPING_ARM_MOTOR_CHANNEL);
        this.winch = provider.getTalon(ElectronicsConstants.CLIMBER_WINCH_MOTOR_CHANNEL);

        this.stower = provider.getDoubleSolenoid(ElectronicsConstants.CLIMBER_STOWER_CHANNEL_A,
            ElectronicsConstants.CLIMBER_STOWER_CHANNEL_B);
        this.leaner = provider.getDoubleSolenoid(ElectronicsConstants.CLIMBER_LEANER_CHANNEL_A,
            ElectronicsConstants.CLIMBER_LEANER_CHANNEL_B);
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

    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.telescopingArm.set(0);
        this.winch.set(0);

        this.stower.set(DoubleSolenoidValue.kOff);
        this.leaner.set(DoubleSolenoidValue.kOff);
    }
}
