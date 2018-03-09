package org.usfirst.frc.team1318.robot.driver.controltasks;

import java.util.Arrays;

import org.usfirst.frc.team1318.robot.ai.Device;
import org.usfirst.frc.team1318.robot.ai.Organism;
import org.usfirst.frc.team1318.robot.ai.RAPID;
import org.usfirst.frc.team1318.robot.ai.RAPIDSettings;
import org.usfirst.frc.team1318.robot.ai.Range;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

public class RAPIDElevatorTask extends RAPID
{
    private RAPIDSettings settings;
    private Device device;
    private IControlTask currentTask;
    private double[] trialPositions;
    private int curOrganism = -1;

    public RAPIDElevatorTask(RAPIDSettings settings, Device device, double[] trialPositions)
    {
        super(settings);
        this.currentTask = null;
        this.device = device;
        this.trialPositions = trialPositions;
    }

    @Override
    public void begin()
    {

        // Initialize population
        sample = new PIDAutoTuneTask[settings.populationSize];
        if (settings.initialValues != null)
        {
            this.initialize(sample, settings.initialValues);
        }
        else
        {
            this.initialize(sample);
        }
        System.out.println("Population initialized!");
    }

    @Override
    public void update()
    {
        // check whether we should continue with the current task
        if (this.currentTask != null)
        {
            if (this.currentTask.hasCompleted())
            {
                this.currentTask.end();
                this.currentTask = null;
            }
        }

        if (this.currentTask == null)  // if there's no current task, find the next one and start it (if any)
        {
            curOrganism++;
            if (curOrganism < this.sample.length)
            {
                this.currentTask = sample[curOrganism];
                this.currentTask.begin();
            }
            else
            {
                curOrganism = -1;
                this.next();
                if (this.hasCompleted())
                {
                    return;
                }
            }
        }
    }

    @Override
    public void stop()
    {
        // TODO Auto-generated method stub
        if (this.currentTask != null)
        {
            this.currentTask.stop();
        }
    }

    @Override
    public void end()
    {
        // TODO Auto-generated method stub
        Arrays.sort(sample);
        System.out.println("\nTop Organisms (Population: " + sample.length + ")");
        System.out.print("1. " + sample[0] + "; ");
        for (int i = 1; i < RAPIDSettings.PRINT_LENGTH; i++)
        {
            System.out.println();
            System.out.print((i + 1) + ". " + sample[i] + "; ");
        }
    }

    @Override
    public boolean shouldCancel()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected Organism getNewOrganism()
    {
        // TODO Auto-generated method stub
        return new PIDAutoTuneTask(device, null, trialPositions);
    }

    @Override
    protected Organism getNewOrganism(Range[] initialValues, Range[] geneBounds)
    {
        return new PIDAutoTuneTask(device, initialValues, settings.geneBounds, trialPositions);
    }
}
