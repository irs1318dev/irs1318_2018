package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

/**
 * Task that holds multiple other tasks and executes them sequentially (in order).
 * 
 */
public class ClimbTask extends SequentialTask implements IControlTask
{
    private ElevatorMechanism elevator;

    /**
     * Initializes a new SequentialTask
     * @param tasks to run
     */
    public ClimbTask()
    {
        super(new IControlTask[] {
            new ReleaseServoTimedTask(2.5),
            new ElevatorMovementTask(1.0, Operation.ElevatorCarryPosition),
            new EnableWinchTimedTask(.2)
        });
    }

    /**
    * Begin the current task
    */
    @Override
    public void begin()
    {
        elevator = this.getInjector().getInstance(ElevatorMechanism.class);

        if (elevator.getInnerPosition() < TuningConstants.ELEVATOR_INNER_CLIMB_POSITION
            && elevator.getOuterPosition() < TuningConstants.ELEVATOR_OUTER_CLIMB_POSITION)
        {
            end();
        }
    }
}
