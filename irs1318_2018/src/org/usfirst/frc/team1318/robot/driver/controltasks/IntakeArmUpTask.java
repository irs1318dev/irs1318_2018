package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

public class IntakeArmUpTask extends TimedTask implements IControlTask
{
    public IntakeArmUpTask(double duration)
    {
        super(duration);
    }

    @Override
    public void begin()
    {
        super.begin();
        this.setDigitalOperationState(Operation.ElevatorIntakeArmsUp, true);
        this.setDigitalOperationState(Operation.ElevatorIntakeArmsDown, false);
    }

    @Override
    public void update()
    {
    }

    public void stop()
    {
        super.stop();
        this.setDigitalOperationState(Operation.ElevatorIntakeArmsUp, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeArmsDown, false);
    }

    public void end()
    {
        super.end();
        this.setDigitalOperationState(Operation.ElevatorIntakeArmsUp, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeArmsDown, false);
    }
}
