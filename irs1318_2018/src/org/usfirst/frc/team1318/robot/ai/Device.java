package org.usfirst.frc.team1318.robot.ai;

public interface Device
{
    public void setTarget(double target);

    // Get positive/negative error from set target
    public double getError();

    // Immediately stop this device
    public void stop();

    // Set PIDF constants on device
    public void setPIDF(double p, double i, double d, double f);

    // Set Control Mode of device
    public void usePID(boolean usePID);
}
