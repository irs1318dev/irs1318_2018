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

    private double startTime;
    private Double outerBeamBrokenTime;

    /**
     * Initializes a new IntakeAndCorrectionTask
     */
    public IntakeAndCorrectionTask()
    {
        this.outerBeamBrokenTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
        this.startTime = this.timer.get();

        this.setDigitalOperationState(Operation.ElevatorIntake, true);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorOuttake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, false);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double timeSinceStart = currentTime - this.startTime;
        boolean shouldFingerIn = timeSinceStart >= TuningConstants.ELEVATOR_FINGER_OUT_TIME_THRESHOLD;
        if (this.elevator.getOuterThroughBeamStatus() && !this.elevator.getInnerThroughBeamStatus())
        {
            if (this.outerBeamBrokenTime == null)
            {
                this.outerBeamBrokenTime = currentTime;
            }

            double waitTime = currentTime - this.outerBeamBrokenTime;
            if (waitTime > TuningConstants.ELEVATOR_INTAKE_CORRECTION_TRIGGER_TIME_THRESHOLD)
            {
                double correctTime = waitTime - TuningConstants.ELEVATOR_INTAKE_CORRECTION_TRIGGER_TIME_THRESHOLD;
                if (correctTime < TuningConstants.ELEVATOR_INTAKE_CORRECTION_OPERATION_TIME_THRESHOLD)
                {
                    this.setDigitalOperationState(Operation.ElevatorIntake, false);
                    this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, true);
                    this.setDigitalOperationState(Operation.ElevatorOuttake, false);
                    this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, shouldFingerIn);

                    return;
                }
                else
                {
                    this.outerBeamBrokenTime = null;
                }
            }
        }
        else
        {
            this.outerBeamBrokenTime = null;
        }

        this.setDigitalOperationState(Operation.ElevatorIntake, true);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorOuttake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, shouldFingerIn);
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
        this.setDigitalOperationState(Operation.ElevatorIntakeFingersIn, false);
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
        return false; //this.elevator.getInnerThroughBeamStatus();
    }
}
