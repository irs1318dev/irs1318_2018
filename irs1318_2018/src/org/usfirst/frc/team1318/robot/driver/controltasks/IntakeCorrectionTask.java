package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

import edu.wpi.first.wpilibj.Timer;

/**
 * Abstract class defining a task that moves the robot a certain distance using Positional PID.
 * 
 */
public abstract class IntakeCorrectionTask extends ControlTaskBase implements IControlTask
{
    private ElevatorMechanism elevator;
    private final Timer timer;
    private Double startTime;
    private Double intermediateTime;

    /**
     * Initializes a new MoveDistanceTaskBase
     * @param resetPositionalOnEnd
     */
    protected IntakeCorrectionTask()
    {
        this.timer = new Timer();
        this.startTime = this.timer.get();
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        if (this.elevator.getOuterThroughBeamStatus() && !this.elevator.getInnerThroughBeamStatus())
        {
            this.intermediateTime = this.timer.get() - this.startTime;
            if (this.intermediateTime > TuningConstants.ELEVATOR_INTAKE_CORRECTION_TRIGGER_TIME_THRESHOLD)
            {
                if (this.intermediateTime
                    - TuningConstants.ELEVATOR_INTAKE_CORRECTION_TRIGGER_TIME_THRESHOLD < TuningConstants.ELEVATOR_INTAKE_CORRECTION_OPERATION_TIME_THRESHOLD)
                {
                    this.setDigitalOperationState(Operation.ElevatorIntake, false);
                    this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, true);
                }
                else
                {
                    this.intermediateTime = 0.0;
                    this.startTime = timer.get();
                }
            }
            else
            {
                this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
                this.setDigitalOperationState(Operation.ElevatorIntake, true);
            }
        }
        else
        {
            this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
            this.setDigitalOperationState(Operation.ElevatorIntake, true);
        }

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
        return this.elevator.getInnerThroughBeamStatus();
    }
}
