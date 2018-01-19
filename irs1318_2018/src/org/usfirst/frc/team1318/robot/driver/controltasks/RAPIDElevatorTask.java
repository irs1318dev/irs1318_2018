package org.usfirst.frc.team1318.robot.driver.controltasks;

import java.util.Arrays;

import org.usfirst.frc.team1318.robot.ai.Organism;
import org.usfirst.frc.team1318.robot.ai.RAPID;
import org.usfirst.frc.team1318.robot.ai.RAPIDSettings;
import org.usfirst.frc.team1318.robot.ai.Range;
import org.usfirst.frc.team1318.robot.common.wpilib.IDigitalInput;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.WpilibProvider;
import org.usfirst.frc.team1318.robot.driver.common.IControlTask;

public class RAPIDElevatorTask extends RAPID
{
    private RAPIDSettings settings;
    private int deviceNumber;
    private int limitSwitchTopChannel;
    private int limitSwitchBottomChannel;
    private ITalonSRX talon;
    private IDigitalInput limitSwitchTop;
    private IDigitalInput limitSwitchBottom;
    private IControlTask currentTask;

    private int curOrganism = -1;

    public RAPIDElevatorTask(RAPIDSettings settings, int deviceNumber, int limitSwitchTopChannel, int limitSwitchBottomChannel)
    {
        super(settings);
        this.currentTask = null;
        this.deviceNumber = deviceNumber;
        this.limitSwitchTopChannel = limitSwitchTopChannel;
        this.limitSwitchBottomChannel = limitSwitchBottomChannel;
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
        for (int i = 1; i < settings.PRINT_LENGTH; i++)
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
        return new PIDAutoTuneTask(talon, limitSwitchTop, limitSwitchBottom, null);
    }

    @Override
    protected Organism getNewOrganism(Range[] initialValues, Range[] geneBounds)
    {
        return new PIDAutoTuneTask(talon, limitSwitchTop, limitSwitchBottom,
            initialValues, settings.geneBounds);
    }
}
