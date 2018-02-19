package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

public class IntakeTask extends TimedTask implements IControlTask
{
    private ElevatorMechanism elevator;

    public IntakeTask(double duration)
    {
        super(duration);
    }

    @Override
    public void begin()
    {
        super.begin();

        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);

        this.setDigitalOperationState(Operation.ElevatorIntake, true);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, true);
    }

    @Override
    public void update()
    {
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, false);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, false);
    }

    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
        {
            return true;
        }

        return this.elevator.getInnerThroughBeamStatus();
    }
}
