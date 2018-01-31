package org.usfirst.frc.team1318.robot.elevator;

import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.Test;
import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TestWpilibProvider;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilib.IAnalogInput;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXLimitSwitchStatus;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXNeutralMode;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

public class ElevatorMechanismTest
{
    @Test
    public void testSetPower_Zero()
    {
        IDashboardLogger logger = mock(IDashboardLogger.class);
        ITimer timer = mock(ITimer.class);
        TestWpilibProvider testProvider = new TestWpilibProvider();
        ITalonSRX innerElevatorMotor = testProvider.getTalonSRX(ElectronicsConstants.ELEVATOR_INNER_MOTOR_CAN_ID);
        ITalonSRX outerElevatorMotor = testProvider.getTalonSRX(ElectronicsConstants.ELEVATOR_OUTER_MOTOR_CAN_ID);
        ITalonSRX leftCarriageIntakeMotor = testProvider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_CARRIAGE_INTAKE_MOTOR_CAN_ID);
        ITalonSRX rightCarriageIntakeMotor = testProvider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_CARRIAGE_INTAKE_MOTOR_CAN_ID);
        ITalonSRX leftOuterIntakeMotor = testProvider.getTalonSRX(ElectronicsConstants.ELEVATOR_LEFT_OUTER_INTAKE_MOTOR_CAN_ID);
        ITalonSRX rightOuterIntakeMotor = testProvider.getTalonSRX(ElectronicsConstants.ELEVATOR_RIGHT_OUTER_INTAKE_MOTOR_CAN_ID);
        IAnalogInput innerThroughBeamSensor = testProvider.getAnalogInput(ElectronicsConstants.ELEVATOR_INNER_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL);
        IAnalogInput outerThroughBeamSensor = testProvider.getAnalogInput(ElectronicsConstants.ELEVATOR_OUTER_THROUGH_BEAM_SENSOR_ANALOG_CHANNEL);
        Driver driver = mock(Driver.class);

        doReturn(0.0).when(innerElevatorMotor).getError();
        doReturn(0.0).when(innerElevatorMotor).getVelocity();
        doReturn(0).when(innerElevatorMotor).getPosition();
        doReturn(new TalonSRXLimitSwitchStatus(false, false)).when(innerElevatorMotor).getLimitSwitchStatus();
        doReturn(0.0).when(outerElevatorMotor).getError();
        doReturn(0.0).when(outerElevatorMotor).getVelocity();
        doReturn(0).when(outerElevatorMotor).getPosition();
        doReturn(new TalonSRXLimitSwitchStatus(false, false)).when(outerElevatorMotor).getLimitSwitchStatus();
        doReturn(0.0).when(innerThroughBeamSensor).getVoltage();
        doReturn(0.0).when(outerThroughBeamSensor).getVoltage();

        ElevatorMechanism elevatorMechanism = new ElevatorMechanism(logger, testProvider, timer);
        elevatorMechanism.setDriver(driver);
        elevatorMechanism.readSensors();
        elevatorMechanism.update();

        // from constructor:
        verify(innerElevatorMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(innerElevatorMotor).setInvertOutput(eq(true));
        verify(innerElevatorMotor).setInvertSensor(eq(true));
        verify(innerElevatorMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(innerElevatorMotor).setSelectedSlot(eq(0));
        verify(innerElevatorMotor).setForwardLimitSwitch(
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        verify(innerElevatorMotor).setReverseLimitSwitch(
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        verify(innerElevatorMotor).setPIDF(
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KP,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KI,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KD,
            TuningConstants.ELEVATOR_POSITION_PID_INNER_KF,
            0);
        verify(innerElevatorMotor).setControlMode(TalonSRXControlMode.Position);
        verify(outerElevatorMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(outerElevatorMotor).setInvertOutput(eq(false));
        verify(outerElevatorMotor).setInvertSensor(eq(false));
        verify(outerElevatorMotor).setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        verify(outerElevatorMotor).setSelectedSlot(eq(0));
        verify(outerElevatorMotor).setForwardLimitSwitch(
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_INNER_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        verify(outerElevatorMotor).setReverseLimitSwitch(
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_OUTER_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        verify(outerElevatorMotor).setPIDF(
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KP,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KI,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KD,
            TuningConstants.ELEVATOR_POSITION_PID_OUTER_KF,
            0);
        verify(outerElevatorMotor).setControlMode(TalonSRXControlMode.Position);

        verify(leftCarriageIntakeMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftCarriageIntakeMotor).setInvertOutput(eq(false));
        verify(leftCarriageIntakeMotor).setControlMode(TalonSRXControlMode.PercentOutput);
        verify(rightCarriageIntakeMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightCarriageIntakeMotor).setInvertOutput(eq(false));
        verify(rightCarriageIntakeMotor).setControlMode(TalonSRXControlMode.PercentOutput);

        verify(leftOuterIntakeMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(leftOuterIntakeMotor).setInvertOutput(eq(false));
        verify(leftOuterIntakeMotor).setControlMode(TalonSRXControlMode.PercentOutput);
        verify(rightOuterIntakeMotor).setNeutralMode(eq(TalonSRXNeutralMode.Brake));
        verify(rightOuterIntakeMotor).setInvertOutput(eq(false));
        verify(rightOuterIntakeMotor).setControlMode(TalonSRXControlMode.PercentOutput);

        // from readSensors:
        verify(innerElevatorMotor).getError();
        verify(innerElevatorMotor).getVelocity();
        verify(innerElevatorMotor).getPosition();
        verify(innerElevatorMotor).getLimitSwitchStatus();
        verify(outerElevatorMotor).getError();
        verify(outerElevatorMotor).getVelocity();
        verify(outerElevatorMotor).getPosition();
        verify(outerElevatorMotor).getLimitSwitchStatus();
        verify(innerThroughBeamSensor).getVoltage();
        verify(outerThroughBeamSensor).getVoltage();

        // from update:
        verify(innerElevatorMotor).set(eq(0.0));
        verify(outerElevatorMotor).set(eq(0.0));
        verify(leftCarriageIntakeMotor).set(eq(0.0));
        verify(rightCarriageIntakeMotor).set(eq(0.0));
        verify(leftOuterIntakeMotor).set(eq(0.0));
        verify(rightOuterIntakeMotor).set(eq(0.0));

        verifyNoMoreInteractions(innerElevatorMotor);
        verifyNoMoreInteractions(outerElevatorMotor);
        verifyNoMoreInteractions(leftCarriageIntakeMotor);
        verifyNoMoreInteractions(rightCarriageIntakeMotor);
        verifyNoMoreInteractions(leftOuterIntakeMotor);
        verifyNoMoreInteractions(rightOuterIntakeMotor);
        verifyNoMoreInteractions(innerThroughBeamSensor);
        verifyNoMoreInteractions(outerThroughBeamSensor);
    }

    @Test
    public void testStop()
    {
    }
}
