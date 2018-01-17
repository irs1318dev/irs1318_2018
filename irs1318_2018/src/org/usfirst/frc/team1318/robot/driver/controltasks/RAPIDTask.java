package org.usfirst.frc.team1318.robot.driver.controltasks;

import java.util.Arrays;

import org.usfirst.frc.team1318.robot.ai.Organism;
import org.usfirst.frc.team1318.robot.ai.RAPIDSettings;
import org.usfirst.frc.team1318.robot.ai.Range;
import org.usfirst.frc.team1318.robot.common.wpilib.IDigitalInput;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.WpilibProvider;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

public class RAPIDTask extends ControlTaskBase
{
    private RAPIDSettings settings;
    private int deviceNumber;
    private int limitSwitchTopChannel;
    private int limitSwitchBottomChannel;
    private ITalonSRX talon;
    private IDigitalInput limitSwitchTop;
    private IDigitalInput limitSwitchBottom;
    private IControlTask currentTask;

    private Organism[] sample;
    private int curGeneration = 0;
    private int lastGeneration = -1;
    private double lastMaxFit = 0;
    private int curOrganism = -1;
    private boolean hasCompleted;

    public RAPIDTask(RAPIDSettings settings, int deviceNumber, int limitSwitchTopChannel, int limitSwitchBottomChannel)
    {
        this.currentTask = null;
        this.settings = settings;
        this.deviceNumber = deviceNumber;
        this.limitSwitchTopChannel = limitSwitchTopChannel;
        this.limitSwitchBottomChannel = limitSwitchBottomChannel;
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
                curGeneration++;
                if (curGeneration >= settings.numGenerations)
                {
                    hasCompleted = true;
                    return;
                }
            }
        }

        // Run through environment for multiple generations
        if (curGeneration < settings.numGenerations && curGeneration != lastGeneration)
        {
            // Display average fitness and max fitness
            if (curGeneration % (settings.numGenerations / 10) == 0)
            {
                double aFit = 0;
                for (Organism o : sample)
                {
                    aFit += o.getFitness();
                }
                aFit /= sample.length;
                System.out.println(
                    "Generation " + curGeneration + "\t Average Fitness: " + aFit + "\t Max Fitness " + sample[0].getFitness());
            }

            // Check for stagnating population
            boolean stagnating = sample[0].getFitness() <= lastMaxFit + settings.stagnationError
                && sample[0].getFitness() >= lastMaxFit - settings.stagnationError;
            lastMaxFit = sample[0].getFitness();
            // Reproduce organisms with highest fitness
            if (stagnating)
            {
                // If sample is stagnating
                sample = this.reproduce(sample, settings.acceleratedMutationRate);
            }
            else
            {
                // If sample is not stagnating
                sample = this.reproduce(sample, settings.mutationRate);
            }
            lastGeneration = curGeneration;
        }

    }

    private void initialize(Organism[] organisms, Range[] initialValues)
    {
        for (int i = 0; i < organisms.length; i++)
        {
            organisms[i] = new PIDAutoTuneTask(talon, limitSwitchTop, limitSwitchBottom,
                initialValues, settings.geneBounds);
        }
    }

    private void initialize(Organism[] organisms)
    {
        // Pick random gene values from the search space to initialize the population.
        this.initialize(organisms, settings.geneBounds);
    }

    // Pick organisms to reproduce
    private Organism[] reproduce(Organism[] o, double mutationRate)
    {
        // Cull organisms not fit to reproduce
        Organism[] survivors = new Organism[settings.populationSize - settings.bottleneckSize];
        for (int i = 0; i < survivors.length; i++)
        {
            survivors[i] = o[i];
        }
        o = survivors;

        // Create new generation of organisms
        Organism[] nextGen = new Organism[settings.populationSize];
        int i = 0;
        while (i < settings.populationSize)
        {
            // Each organism reproduces with another random organism from the surviving pool
            // until population limit is reached

            // Create an array to randomly choose another random organism that isn't this
            // organism.
            // Organisms can't reproduce with themselves.
            int[] pickList = new int[Math.round((float)o.length - 1)];
            Range range = new Range(0, pickList.length - 1);
            int w = 0;
            for (int j = 0; j < pickList.length; j++)
            {
                if (j == i % (settings.populationSize - settings.bottleneckSize))
                {
                    w++;
                }
                pickList[j] = w;
                w++;
            }
            nextGen[i] = new PIDAutoTuneTask(talon, limitSwitchTop, limitSwitchBottom, null);
            Organism.reproduce(o[i % o.length], o[pickList[Math.round((float)range.getRandom())]], mutationRate, nextGen[i]);
            i++;
        }

        return nextGen;
    }

    @Override
    public void begin()
    {
        // TODO Auto-generated method stub
        WpilibProvider provider = this.getInjector().getInstance(WpilibProvider.class);
        limitSwitchTop = provider.getDigitalInput(limitSwitchTopChannel);
        limitSwitchBottom = provider.getDigitalInput(limitSwitchBottomChannel);
        talon = provider.getTalonSRX(deviceNumber);

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
    public void stop()
    {
        // TODO Auto-generated method stub

    }

    @Override
    public void end()
    {
        // TODO Auto-generated method stub
        Arrays.sort(sample);
        System.out.println("\nTop Organisms (Population: " + sample.length + ")");
        System.out.print("1. " + sample[0] + "; ");
        for (int i = 1; i < settings.PRINT_LENGTH; i++)
        {
            System.out.println();
            System.out.print((i + 1) + ". " + sample[i] + "; ");
        }
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
}
