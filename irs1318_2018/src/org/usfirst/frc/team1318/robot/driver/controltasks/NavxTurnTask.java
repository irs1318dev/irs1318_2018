package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.Helpers;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;
import org.usfirst.frc.team1318.robot.general.PositionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class NavxTurnTask extends ControlTaskBase implements IControlTask
{
    private final boolean useTime;
    private final double desiredAngle;
    private final double minRange;
    private final double maxRange;

    private PIDHandler turnPidHandler;
    private Double completeTime;
    protected PositionManager pManager;

    /**
    * Initializes a new NavxTurnTask using time to make sure we completed turn
    * @param desiredAngle the desired angle
    */
    public NavxTurnTask(double desiredAngle)
    {
        this(true, desiredAngle);
    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param desiredAngle the desired angle
    */
    public NavxTurnTask(boolean useTime, double desiredAngle)
    {
        this(
            useTime,
            desiredAngle,
            TuningConstants.NAVX_TURN_MIN_ACCEPTABLE_ANGLE_VALUE,
            TuningConstants.NAVX_TURN_MAX_ACCEPTABLE_ANGLE_VALUE);
    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param desiredAngle the desired angle
    * @param minRange the minimum of the measured angle range that we accept from the navx
    * @param maxRange the maximum of the measured angle range that we accept from the navx
    */
    public NavxTurnTask(boolean useTime, double desiredAngle, double minRange, double maxRange)
    {
        this.useTime = useTime;
        this.desiredAngle = desiredAngle;
        this.minRange = minRange;
        this.maxRange = maxRange;

        this.turnPidHandler = null;
        this.completeTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.pManager = this.getInjector().getInstance(PositionManager.class);
        this.turnPidHandler = this.createTurnHandler();
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);

        double currentMeasuredAngle = this.pManager.getNavxAngle();

        // if we are not within the expected range, let's fall back to using odometry
        if (Helpers.WithinRange(currentMeasuredAngle, this.minRange, this.maxRange))
        {
            currentMeasuredAngle = this.pManager.getOdometryAngle();
        }

        this.setAnalogOperationState(
            Operation.DriveTrainTurn,
            this.turnPidHandler.calculatePosition(desiredAngle, currentMeasuredAngle));
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);

    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double currentMeasuredAngle = this.pManager.getNavxAngle();

        double centerAngleDifference = Math.abs(currentMeasuredAngle - this.desiredAngle);
        if (centerAngleDifference > TuningConstants.MAX_NAVX_TURN_RANGE_DEGREES)
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }
        else
        {
            ITimer timer = this.getInjector().getInstance(ITimer.class);
            if (this.completeTime == null)
            {
                this.completeTime = timer.get();
                return false;
            }
            else if (timer.get() - this.completeTime < TuningConstants.NAVX_TURN_COMPLETE_TIME)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    @Override
    public boolean shouldCancel()
    {
        return false;
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.NAVX_TURN_PID_KP,
            TuningConstants.NAVX_TURN_PID_KI,
            TuningConstants.NAVX_TURN_PID_KD,
            TuningConstants.NAVX_TURN_PID_KF,
            TuningConstants.NAVX_TURN_PID_KS,
            TuningConstants.NAVX_TURN_PID_MIN,
            TuningConstants.NAVX_TURN_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}
