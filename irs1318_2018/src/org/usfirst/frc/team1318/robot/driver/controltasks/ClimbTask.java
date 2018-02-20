package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

/**
 * Task that holds multiple other tasks and executes them sequentially (in order).
 * 
 */
public class ClimbTask extends SequentialTask implements IControlTask
{
    /**
     * Initializes a new SequentialTask
     * @param tasks to run
     */
    public ClimbTask()
    {
        super(new IControlTask[] {
            new ElevatorMovementTask(1.0, Operation.ElevatorClimbPosition),
            new ReleaseServoTimedTask(2.5),
            new ElevatorMovementTask(1.0, Operation.ElevatorCarryPosition),
            new EnableWinchTimedTask(.2)
        });
    }
}
