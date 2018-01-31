package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

/**
 * Class defining a task that intakes a power cube, rotating it if it gets stuck
 * 
 */
public class IntakeAndCorrectionTask extends ControlTaskBase implements IControlTask
{
    private ITimer timer;
    private ElevatorMechanism elevator;

    private Double startTime;

    /**
     * Initializes a new IntakeAndCorrectionTask
     */
    public IntakeAndCorrectionTask()
    {
        this.startTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);

        this.setDigitalOperationState(Operation.ElevatorIntake, true);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorOuttake, false);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        if (this.elevator.getOuterThroughBeamStatus() && !this.elevator.getInnerThroughBeamStatus())
        {
            double currentTime = this.timer.get();
            if (this.startTime == null)
            {
                this.startTime = currentTime;
            }

            double waitTime = currentTime - this.startTime;
            if (waitTime > TuningConstants.ELEVATOR_INTAKE_CORRECTION_TRIGGER_TIME_THRESHOLD)
            {
                double correctTime = waitTime - TuningConstants.ELEVATOR_INTAKE_CORRECTION_TRIGGER_TIME_THRESHOLD;
                if (correctTime < TuningConstants.ELEVATOR_INTAKE_CORRECTION_OPERATION_TIME_THRESHOLD)
                {
                    this.setDigitalOperationState(Operation.ElevatorIntake, false);
                    this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, true);
                    this.setDigitalOperationState(Operation.ElevatorOuttake, false);

                    return;
                }
                else
                {
                    this.startTime = null;
                }
            }
        }
        else
        {
            this.startTime = null;
        }

        this.setDigitalOperationState(Operation.ElevatorIntake, true);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorOuttake, false);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorOuttake, false);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorOuttake, false);
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
