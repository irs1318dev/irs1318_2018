package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

/**
 * Class defining a task that intakes a power cube, rotating it if it gets stuck
 * 
 */
public class ElevatorMovementTask extends TimedTask implements IControlTask
{
    private ElevatorMechanism elevator;

    private boolean completeWithTime; // Either wait for movement completion or operate for a certain period of time
    private boolean moveUpToClimb;

    /**
     * Initializes a new IntakeAndCorrectionTask
     */
    public ElevatorMovementTask(boolean completeWithTime, boolean moveUpToClimb)
    {
        super(TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD);
        this.completeWithTime = completeWithTime;
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
            this.setDigitalOperationState(Operation.ElevatorClimbPosition, true);
            this.setDigitalOperationState(Operation.ElevatorBottomPosition, false);
        }
        else
        {
            this.setDigitalOperationState(Operation.ElevatorBottomPosition, true);
            this.setDigitalOperationState(Operation.ElevatorClimbPosition, false);
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
        this.setDigitalOperationState(Operation.ElevatorClimbPosition, false);
        this.setDigitalOperationState(Operation.ElevatorBottomPosition, false);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.ElevatorClimbPosition, false);
        this.setDigitalOperationState(Operation.ElevatorBottomPosition, false);
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
        if (completeWithTime && super.hasCompleted())
        {
            return true;
        }

        double totalError = elevator.getTotalError();

        if (totalError < TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_DISTANCE_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
