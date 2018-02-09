package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;

public class ReleaseServoTimedTask extends TimedTask
{

    public ReleaseServoTimedTask(double duration)
    {
        super(duration);
    }

    @Override
    public void begin()
    {
        super.begin();
        this.setDigitalOperationState(Operation.ClimberRelease, true);
    }

    @Override
    public void update()
    {

    }
}
