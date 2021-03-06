package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.elevator.ElevatorMechanism;

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
    private final boolean completeWhenThroughBeamBroken;
    private final double completionTimeout;

    private ITimer timer;
    private ElevatorMechanism elevator;

    private double startTime;
    private Double completeTime;
    private boolean areElevatorArmsDown;

    /**
     * Initializes a new AdvancedIntakeOuttakeTask
     */
    public AdvancedIntakeOuttakeTask(Operation intakeOuttakeOperation)
    {
        this(intakeOuttakeOperation, false, 0.25);
    }

    /**
     * Initializes a new AdvancedIntakeOuttakeTask
     */
    public AdvancedIntakeOuttakeTask(Operation intakeOuttakeOperation, boolean completeWhenThroughBeamBroken)
    {
        this(intakeOuttakeOperation, completeWhenThroughBeamBroken, 0.25);
    }

    /**
     * Initializes a new AdvancedIntakeOuttakeTask
     */
    public AdvancedIntakeOuttakeTask(Operation intakeOuttakeOperation, boolean completeWhenThroughBeamBroken, double completionTimeout)
    {
        this.intakeOuttakeOperation = intakeOuttakeOperation;
        this.completeWhenThroughBeamBroken = completeWhenThroughBeamBroken;
        this.completionTimeout = completionTimeout;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = this.timer.get();
        this.completeTime = null;

        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
        this.areElevatorArmsDown = this.elevator.getArmDownStatus();

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
        if (!this.areElevatorArmsDown
            || timeSinceStart >= TuningConstants.ELEVATOR_FINGER_IN_INTAKE_TIME_THRESHOLD)
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
        if (!this.completeWhenThroughBeamBroken || !this.elevator.getThroughBeamStatus())
        {
            this.completeTime = null;
            return false;
        }

        double currentTime = this.timer.get();
        if (this.completeTime == null)
        {
            this.completeTime = currentTime;
            return false;
        }

        return currentTime - this.completeTime > this.completionTimeout;
    }

    private void setOperation(boolean clear)
    {
        for (Operation op : AdvancedIntakeOuttakeTask.allOperations)
        {
            this.setDigitalOperationState(op, !clear && op == this.intakeOuttakeOperation);
        }
    }
}
