package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

public class OuttakeTask extends TimedTask implements IControlTask
{
    public final boolean strong;

    public OuttakeTask(double duration, boolean strong)
    {
        super(duration);
        this.strong = strong;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorWeakOuttake, !this.strong);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, this.strong);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorWeakOuttake, !this.strong);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, this.strong);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorWeakOuttake, false);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, false);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.ElevatorIntake, false);
        this.setDigitalOperationState(Operation.ElevatorIntakeCorrection, false);
        this.setDigitalOperationState(Operation.ElevatorWeakOuttake, false);
        this.setDigitalOperationState(Operation.ElevatorStrongOuttake, false);
    }
}
