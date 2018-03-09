package org.usfirst.frc.team1318.robot.ai;

public class RAPIDSettings
{
    public final Range[] initialValues; // Set to null to populate randomly

    // Genetic Factors
    public final double mutationRate; // Chance that any given gene will mutate when reproducing
    public final double acceleratedMutationRate; // Chance to mutate when stagnation occurs
    public final double stagnationError; // Maximum deviation to classify stagnation

    // Population Factors
    public final int numGenerations; // Number of generations
    public final int populationSize; // Size of sample
    public final int bottleneckSize; // Number of organisms to kill each generation -- must be
                                    // less than population size.

    // Search Space
    public final Range[] geneBounds; // Bound limits for each gene (array size should equal # of genes)

    // User Interface
    public static final int PRINT_LENGTH = 5; // Number of organisms to display at the end.

    public RAPIDSettings(Range[] geneBounds)
    {
        this(null, geneBounds);
    }

    public RAPIDSettings(Range[] initialValues, Range[] geneBounds)
    {
        this(initialValues, 0.05, 0.15, 0.01, 10, 30, 19, geneBounds);
    }

    public RAPIDSettings(
        Range[] initialValues, double mutationRate, double acceleratedMutationRate,
        double stagnationError, int numGenerations, int populationSize, int bottleneckSize, Range[] geneBounds)
    {
        this.initialValues = initialValues;
        this.mutationRate = mutationRate;
        this.acceleratedMutationRate = acceleratedMutationRate;
        this.stagnationError = stagnationError;
        this.numGenerations = numGenerations;
        this.populationSize = populationSize;
        this.bottleneckSize = bottleneckSize;
        this.geneBounds = geneBounds;
    }

}
