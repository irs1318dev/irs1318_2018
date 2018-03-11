package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

/**
 * Class defining a task that intakes or outtakes a power cube
 * 
 */
public class AdvancedIntakeOuttakeTask extends ControlTaskBase implements IControlTask
{
    private static final Operation[] allOperations = new Operation[] {
        Operation.ElevatorIntake,
        Operation.ElevatorIntakeCorrection,
        Operation.ElevatorWeakOuttake,
        Operation.ElevatorStrongOuttake,
    };

    private final Operation intakeOuttakeOperation;

    private ITimer timer;
    private double startTime;

    /**
     * Initializes a new AdvancedIntakeOuttakeTask
     */
    public AdvancedIntakeOuttakeTask(Operation intakeOuttakeOperation)
    {
        this.intakeOuttakeOperation = intakeOuttakeOperation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = this.timer.get();

        this.setOperation(true);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, true);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double timeSinceStart = currentTime - this.startTime;
        if (timeSinceStart >= TuningConstants.ELEVATOR_FINGER_IN_INTAKE_TIME_THRESHOLD)
        {
            this.setOperation(false);
        }

        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, true);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        this.setOperation(true);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, false);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setOperation(true);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, false);
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
        return false;
    }

    private void setOperation(boolean clear)
    {
        for (Operation op : AdvancedIntakeOuttakeTask.allOperations)
        {
            this.setDigitalOperationState(op, !clear && op == this.intakeOuttakeOperation);
        }
    }
}
