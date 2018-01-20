package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.ai.Genome;
import org.usfirst.frc.team1318.robot.ai.Organism;
import org.usfirst.frc.team1318.robot.ai.Range;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;

import edu.wpi.first.wpilibj.Timer;

public class PIDAutoTuneTask extends Organism
{
    private Timer timer;

    private double cumulativeError; // Cumulative error over time
    private double lastTime; // Last time this update loop was run
    private ITalonSRX talon; // Talon to test
    private boolean limitSwitchTop; // Status of top limit switch
    private boolean limitSwitchBottom; // Status of bottom limit switch

    private double[] overshoot; // Contains maximum overshoot for each trial
    private double[] stableTimes; // Contains time for talon to stabilize for each trial
    private Timer stabilityTimer; // Keeps track of time it takes for talon to stabilize

    private boolean stable = false; // Whether or not the talon is currently stable
    private boolean hasCompleted = false; // Whether or not this task has completed
    private boolean unfit = false; // Whether or not the organism is unfit (used to auto-disqualify organisms)

    private enum Stage
    {
        SETUP, RUN_TRIALS, ENDSTEP
    };

    private Stage currentStage; // Current stage of testing
    private int curTrial; // Keeps track of which trial we are currently testing
    private double[] trialPositions; // Position to set elevator at for each trial

    // Initialize this organism with a set of initial ranges to choose from
    public PIDAutoTuneTask(
        ITalonSRX talon,
        Range[] initial, Range[] geneBounds, double[] trialPositions)
    {
        super(initial, geneBounds);
        this.talon = talon;
        this.trialPositions = trialPositions;
    }

    // Initialize this organism with a genome
    public PIDAutoTuneTask(
        ITalonSRX talon,
        Genome genome, double[] trialPositions)
    {
        super(genome);
        this.talon = talon;
        this.trialPositions = trialPositions;
    }

    @Override
    public void begin()
    {
        this.currentStage = Stage.SETUP;

        this.timer = new Timer();
        this.stabilityTimer = new Timer();
        timer.reset();
        timer.start();
        stabilityTimer.reset();
        stabilityTimer.start();

        this.talon.setControlMode(TalonSRXControlMode.PercentOutput);
        this.cumulativeError = 0;
        this.curTrial = 0;

        this.stableTimes = new double[this.trialPositions.length];
        this.overshoot = new double[this.trialPositions.length];
    }

    @Override
    public void update()
    {
        this.limitSwitchBottom = this.getLimitReverse();
        this.limitSwitchTop = this.getLimitForward();

        // Every time the elevator hits the bottom switch, reset the talon.
        if (limitSwitchBottom)
        {
            talon.reset();
        }

        // If the elevator hits the bottom or top while not in setup, reject this organism
        if ((limitSwitchBottom || limitSwitchTop)
            && this.currentStage != Stage.SETUP)
        {
            this.endStep();
            talon.stop();

            // This organism is not fit
            this.unfit = true;
        }

        switch (currentStage)
        {
            case SETUP:
                this.setup();
                break;
            case RUN_TRIALS:
                this.runTrial();
                break;
            case ENDSTEP:
                this.endStep();
                break;
        }

    }

    private void setup()
    {
        talon.set(-0.2); // Set talon to bottom
        if (limitSwitchBottom || limitSwitchTop)
        {
            talon.setControlMode(TalonSRXControlMode.Position);
            Genome g = this.getGenome();
            talon.setPIDF(g.getGene(0), g.getGene(1), g.getGene(2), g.getGene(3), 0);

            currentStage = Stage.RUN_TRIALS;
            timer.reset();
            timer.start();

            stabilityTimer.reset();
            stabilityTimer.start();
        }
    }

    private void runTrial()
    {
        // Set the talon to the current trial position
        talon.set(this.trialPositions[curTrial]);

        // Calculate the cumulative error for fitness calculations
        this.cumulativeError += Math.abs(talon.getError()) * (timer.get() - lastTime);
        lastTime = timer.get();

        // OVERSHOOT CHECK:
        if (curTrial >= 1)
        {
            double lastTrialPosition = this.trialPositions[curTrial - 1];
            // If this position is higher than the last one, find maximum positive overshoot. 
            // Else, find max negative overshoot
            if (this.trialPositions[curTrial] - lastTrialPosition > 0)
            {
                positiveOvershootCheck();
            }
            else
            {
                negativeOvershootCheck();
            }
        }
        else
        {
            positiveOvershootCheck();
        }
        // STABILIZATION CHECK:
        // If the motor has stabilized, set the stabilization flag to true.
        if (talon.getError() < TuningConstants.AI_MAX_STABILIZATION_ERROR)
        {
            stable = true;
        }

        // If the motor is not stabilized, set the stabilization flag to false and move the firstStableTime
        if (talon.getError() > TuningConstants.AI_MAX_STABILIZATION_ERROR)
        {
            stableTimes[curTrial] = stabilityTimer.get();
            stable = false;
        }

        // If motor remains stable for STABILIZATION_TIME seconds, go to next trial.
        if (stable && stabilityTimer.get() - stableTimes[curTrial] >= TuningConstants.AI_STABILIZATION_TIME)
        {
            this.nextTrial();
            return;
        }

        // Automatically move to next trial if the period has passed
        if (timer.hasPeriodPassed(TuningConstants.AI_TUNING_SAMPLE_TIME))
        {
            this.nextTrial();
            return;
        }
    }

    private void endStep()
    {
        talon.stop();
        this.hasCompleted = true;
    }

    // Find maximum negative error
    private void negativeOvershootCheck()
    {
        if (talon.getError() < overshoot[curTrial])
        {
            overshoot[curTrial] = talon.getError();
        }

    }

    // Find maximum positive error
    private void positiveOvershootCheck()
    {
        if (talon.getError() > overshoot[curTrial])
        {
            overshoot[curTrial] = talon.getError();
        }
    }

    private void nextTrial()
    {
        curTrial++;
        if (curTrial >= trialPositions.length)
        {
            this.currentStage = Stage.ENDSTEP;
        }
        stable = false;
        timer.reset();
        timer.start();

        stabilityTimer.reset();
        stabilityTimer.start();
    }

    @Override
    public void stop()
    {
        // TODO Auto-generated method stub
        currentStage = Stage.ENDSTEP;
        talon.stop();
    }

    @Override
    public void end()
    {
        // TODO Auto-generated method stub
        this.setFitness();
    }

    @Override
    public boolean hasCompleted()
    {
        // TODO Auto-generated method stub
        return hasCompleted;
    }

    @Override
    public boolean shouldCancel()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setFitness()
    {
        double aOvershoot = 0; // Average overshoot
        double aStabilization = 0; // Average stabilization time

        for (double o : overshoot)
        {
            aOvershoot += Math.abs(o); // Overshoot will be filled with negative values -- make them positive.
        }
        aOvershoot /= overshoot.length;

        for (double s : stableTimes)
        {
            aStabilization += s;
        }
        aStabilization /= stableTimes.length;

        if (!unfit)
        {
            // Higher overshoot and time for motors to stabilize means less fitness.
            this.fitness = 100 / ((aOvershoot * TuningConstants.AI_OVERSHOOT_WEIGHT + aStabilization) + 1);
        }
        else
        {
            // If this organism is unfit, kill it. Let the past die. It's the only way you can become who you were meant to be.
            this.fitness = -Double.MAX_VALUE;
        }
    }

    private boolean getLimitForward()
    {
        return this.talon.getLimitSwitchStatus().isForwardClosed;
    }

    private boolean getLimitReverse()
    {
        return this.talon.getLimitSwitchStatus().isReverseClosed;
    }

}
