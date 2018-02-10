package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;

public class EnableWinchTimedTask extends TimedTask
{

    public EnableWinchTimedTask(double duration)
    {
        super(duration);
    }

    @Override
    public void begin()
    {
        super.begin();
        this.setDigitalOperationState(Operation.ClimberEnableWinch, true);
    }

    @Override
    public void update()
    {

    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.ClimberEnableWinch, false);
    }

    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.ClimberEnableWinch, false);
    }
}
