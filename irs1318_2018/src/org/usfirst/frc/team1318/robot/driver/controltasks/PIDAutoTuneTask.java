package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.ai.Genome;
import org.usfirst.frc.team1318.robot.ai.Organism;
import org.usfirst.frc.team1318.robot.ai.Range;
import org.usfirst.frc.team1318.robot.common.wpilib.IDigitalInput;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;

import edu.wpi.first.wpilibj.Timer;

public class PIDAutoTuneTask extends Organism
{
    private ITalonSRX talon;
    private IDigitalInput limitTop;
    private IDigitalInput limitBottom;
    private boolean hasCompleted = false;
    private double cumulativeError;
    private Timer timer;

    private double lastTime;

    private enum STAGE
    {
        SETUP, STEP1, STEP2, STEP3, ENDSTEP
    };

    private STAGE currentStage;

    // Initialize this organism with a set of initial ranges to choose from
    public PIDAutoTuneTask(
        ITalonSRX talon, IDigitalInput limitTop, IDigitalInput limitBottom,
        Range[] initial, Range[] geneBounds)
    {
        super(initial, geneBounds);
        this.limitBottom = limitBottom;
        this.limitTop = limitTop;
        this.talon = talon;
    }

    // Initialize this organism with a genome
    public PIDAutoTuneTask(
        ITalonSRX talon, IDigitalInput limitTop, IDigitalInput limitBottom,
        Genome genome)
    {
        super(genome);
        this.limitBottom = limitBottom;
        this.limitTop = limitTop;
        this.talon = talon;
    }

    @Override
    public void begin()
    {
        this.currentStage = STAGE.SETUP;
        this.timer = new Timer();
        this.talon.changeControlMode(TalonSRXControlMode.PercentOutput);
        this.cumulativeError = 0;
    }

    @Override
    public void update()
    {
        if (currentStage != STAGE.SETUP && currentStage != STAGE.ENDSTEP)
        {
            cumulativeError += Math.abs(talon.getError()) * (timer.get() - lastTime);
        }
        switch (currentStage)
        {
            case SETUP:
                this.setup();
                break;
            case STEP1:
                this.step1();
                break;
            case STEP2:
                this.step2();
                break;
            case STEP3:
                this.step3();
                break;
            case ENDSTEP:
                this.endStep();
                break;
        }
        lastTime = timer.get();

    }

    private void setup()
    {
        // TODO Auto-generated method stub
        talon.set(-0.2); // Set talon to bottom
        if (limitBottom.get()) // If the bottom limit switch is triggered
        {
            Genome g = this.getGenome();
            talon.changeControlMode(TalonSRXControlMode.Position);
            talon.setPIDF(g.getGene(0), g.getGene(1), g.getGene(2), g.getGene(3), 0);

            currentStage = STAGE.STEP1;
            timer.reset();
            timer.start();
        }
    }

    private void step1()
    {
        talon.set(0.5);
        if (timer.hasPeriodPassed(TuningConstants.AI_TUNING_SAMPLE_TIME))
        {
            currentStage = STAGE.STEP2;
        }
    }

    private void step2()
    {
        talon.set(0.2);
        if (timer.hasPeriodPassed(TuningConstants.AI_TUNING_SAMPLE_TIME))
        {
            currentStage = STAGE.STEP3;
        }
    }

    private void step3()
    {
        talon.set(0.8);
        if (timer.hasPeriodPassed(TuningConstants.AI_TUNING_SAMPLE_TIME))
        {
            currentStage = STAGE.ENDSTEP;
        }
    }

    private void endStep()
    {
        this.hasCompleted = true;
    }

    @Override
    public void stop()
    {
        // TODO Auto-generated method stub

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
        // TODO Auto-generated method stub
        this.fitness = 1 / (cumulativeError + 1);
    }

}
