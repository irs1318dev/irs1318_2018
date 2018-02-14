package org.usfirst.frc.team1318.robot.common.wpilib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorSPXWrapper implements IVictorSPX
{
    private final VictorSPX wrappedObject;

    private ControlMode controlMode;

    public VictorSPXWrapper(int deviceNumber)
    {
        this.wrappedObject = new VictorSPX(deviceNumber);
        this.controlMode = ControlMode.PercentOutput;
    }

    public void set(double value)
    {
        this.wrappedObject.set(this.controlMode, value);
    }

    public void setInvertOutput(boolean invert)
    {
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        this.wrappedObject.setSensorPhase(invert);
    }
}
