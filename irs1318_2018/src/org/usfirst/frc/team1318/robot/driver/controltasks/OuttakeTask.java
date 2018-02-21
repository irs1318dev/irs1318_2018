package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

public class OuttakeTask extends TimedTask implements IControlTask
{
    public OuttakeTask(double duration)
    {
        super(duration);
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, true);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, true);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, false);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, false);
    }
}
