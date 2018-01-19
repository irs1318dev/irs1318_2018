package org.usfirst.frc.team1318.robot.ai;

import org.usfirst.frc.team1318.robot.driver.controltasks.ControlTaskBase;

public abstract class RAPID extends ControlTaskBase
{
    protected Organism[] sample;
    private RAPIDSettings settings;
    private int curGeneration = 0;
    private double lastMaxFit;

    private boolean done;

    public RAPID(RAPIDSettings settings)
    {
        this.settings = settings;
    }

    public void next()
    {
        // Run through environment for multiple generations
        if (curGeneration < settings.numGenerations)
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
            curGeneration++;
        }
        if (curGeneration <= settings.numGenerations)
        {
            this.done = true;
        }

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
            nextGen[i] = this.getNewOrganism();
            Organism.reproduce(o[i % o.length], o[pickList[Math.round((float)range.getRandom())]], mutationRate, nextGen[i]);
            i++;
        }

        return nextGen;
    }

    @Override
    public boolean hasCompleted()
    {
        // TODO Auto-generated method stub
        return done;
    }

    public void initialize(Organism[] organisms, Range[] initialValues)
    {
        for (int i = 0; i < organisms.length; i++)
        {
            organisms[i] = this.getNewOrganism(initialValues, settings.geneBounds);
        }
    }

    public void initialize(Organism[] organisms)
    {
        // Pick random gene values from the search space to initialize the population.
        this.initialize(organisms, settings.geneBounds);
    }

    protected abstract Organism getNewOrganism();

    protected abstract Organism getNewOrganism(Range[] initialValues, Range[] geneBounds);

}
