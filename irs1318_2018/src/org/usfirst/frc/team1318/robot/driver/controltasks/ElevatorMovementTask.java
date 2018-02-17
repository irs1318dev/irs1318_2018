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
    private static final Operation[] AllElevatorPositionOperations = new Operation[] {
        Operation.ElevatorBottomPosition,
        Operation.ElevatorCarryPosition,
        Operation.ElevatorSwitchPosition,
        Operation.ElevatorLowScalePosition,
        Operation.ElevatorHighScalePosition,
        Operation.ElevatorClimbPosition,
        Operation.ElevatorTopPosition,
    };

    private ElevatorMechanism elevator;

    private final boolean completeWithTime; // Either wait for movement completion or operate for a certain period of time
    private final Operation desiredElevatorPositionOperation;

    public ElevatorMovementTask(Operation desiredElevatorPositionOperation)
    {
        super(TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD);

        this.completeWithTime = false;
        this.desiredElevatorPositionOperation = desiredElevatorPositionOperation;
    }

    public ElevatorMovementTask(double duration, Operation desiredElevatorPositionOperation)
    {
        super(duration);

        this.completeWithTime = true;
        this.desiredElevatorPositionOperation = desiredElevatorPositionOperation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);

        for (Operation op : ElevatorMovementTask.AllElevatorPositionOperations)
        {
            this.setDigitalOperationState(op, op == this.desiredElevatorPositionOperation);
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
        super.stop();

        for (Operation op : ElevatorMovementTask.AllElevatorPositionOperations)
        {
            this.setDigitalOperationState(op, false);
        }
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        for (Operation op : ElevatorMovementTask.AllElevatorPositionOperations)
        {
            this.setDigitalOperationState(op, false);
        }
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
        if (this.completeWithTime && super.hasCompleted())
        {
            return true;
        }

        double totalError = this.elevator.getTotalError();
        if (totalError < TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_DISTANCE_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
