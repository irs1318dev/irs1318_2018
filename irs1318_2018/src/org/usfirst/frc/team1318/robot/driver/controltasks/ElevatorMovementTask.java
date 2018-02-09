package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

/**
 * Class defining a task that intakes a power cube, rotating it if it gets stuck
 * 
 */
public class ElevatorMovementTask extends ControlTaskBase implements IControlTask
{
    private ElevatorMechanism elevator;

    private boolean completeInRange; // Either wait for movement completion or complete task once within range
    private boolean moveUpToClimb;

    private double destinationLocation;

    /**
     * Initializes a new IntakeAndCorrectionTask
     */
    public ElevatorMovementTask(boolean completeInRange, boolean moveToHighestPosition)
    {
        this.completeInRange = completeInRange;
        this.moveUpToClimb = moveUpToClimb;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        if (this.moveUpToClimb)
        {
            this.setDigitalOperationState(Operation.ElevatorTopPosition, true);
        }
        else
        {
            this.setDigitalOperationState(Operation.ElevatorBottomPosition, true);
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {

    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
    }

    /**
     * Checks whether this task should be stopped, or whether it should continue being processed.
     * @return true if we should cancel this task (and stop performing any subsequent tasks), otherwise false (to keep processing this task)
     */
    @Override
    public boolean shouldCancel()
    {
        return false;
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        //TODO: Complete this method
        return false;
    }
}
