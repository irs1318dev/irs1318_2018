package org.usfirst.frc.team1318.robot.ai;

import org.usfirst.frc.team1318.robot.driver.controltasks.ControlTaskBase;

public abstract class Organism extends ControlTaskBase implements Comparable<Organism>
{
    private Genome genome;
    protected double fitness;

    // Generate random organism within the search space
    public Organism(Range[] initial, Range[] geneBounds)
    {
        double[] genes = new double[initial.length];

        for (int i = 0; i < genes.length; i++)
        {
            genes[i] = initial[i].getRandom();
        }

        this.genome = new Genome(genes, geneBounds);
    }

    public Organism(Genome genome)
    {
        this.genome = genome;
    }

    public static void reproduce(Organism w, Organism o, double mutationRate, Organism child)
    {
        Genome c = w.genome.combine(o.getGenome(), mutationRate);
        child.genome = c;
    }

    // Sets the fitness of this organism
    public abstract void setFitness();

    public double getFitness()
    {
        return this.fitness;
    }

    public Genome getGenome()
    {
        return this.genome;
    }

    @Override
    public String toString()
    {
        return genome.toString() + ": (" + this.getFitness() + ")";
    }

    @Override
    public int compareTo(Organism o)
    {
        if (this.fitness < o.fitness)
        {
            return 1;
        }
        else if (o.fitness < this.fitness)
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }
}
