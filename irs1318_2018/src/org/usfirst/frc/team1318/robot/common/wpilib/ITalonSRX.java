package org.usfirst.frc.team1318.robot.common.wpilib;

public interface ITalonSRX extends IMotor
{
    void setControlMode(TalonSRXControlMode mode);
    void setSensorType(TalonSRXFeedbackDevice feedbackDevice);
    void setSelectedSlot(int slotId);
    void setPIDF(double p, double i, double d, double f, int slotId);
    void setMotionMagicPIDF(double p, double i, double d, double f, int velocity, int acceleration, int slotId);
    void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId);
    void setForwardLimitSwitch(boolean enabled, boolean normallyOpen);
    void setReverseLimitSwitch(boolean enabled, boolean normallyOpen);
    void setInvertOutput(boolean flip);
    void setInvertSensor(boolean flip);
    void setNeutralMode(TalonSRXNeutralMode neutralMode);
    void setVoltageCompensation(boolean enabled, double maxVoltage);
    void stop();
    void setPosition(int position);
    void reset();
    int getPosition();
    double getVelocity();
    double getError();
    TalonSRXLimitSwitchStatus getLimitSwitchStatus();
}
