package frc.robot.subsystems;

import java.lang.Math;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless); // read parameters from Constants.java
    private final AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private final double maxAngle = 0;
    private final double minAngle = 75;
    private final double kp = 0.01;
    private final double minPowerAtLevel = 0.025;
    private final PIDController pid = new PIDController(kp, 0.0, 0.0);
    private double setpoint = minAngle;
    private final double setpointIncrementer = 0.5;
    private double motorOutput = 0.0;
    private final double maxPIDSpeed = 0.3;
    private double downSpeed = -0.2;

    // settings
    private final boolean usingPID = true;
    private final boolean settingMinLevel = true;   

    public ArmSubsystem() {
        pid.setTolerance(10.0);

        motor.setInverted(false);
        setSetpoint(minAngle);
    }

    public void raise() {
        if (usingPID) {
            setSetpoint(setpoint + setpointIncrementer);
        } else {
            motorOutput = 0.2;
        }
    }

    public void lower() {
        if (usingPID) {
            setSetpoint(setpoint - setpointIncrementer);
        } else {
            motorOutput = -0.2;
        }
    }

    // simply stop
    public void stop() {
        motorOutput = 0.0;
    }

    public void setSetpoint(double newSetpointValue) {
        if (newSetpointValue > maxAngle) {
            setpoint = maxAngle;
        } else if (newSetpointValue < minAngle) {
            setpoint = minAngle;
        } else {
            setpoint = newSetpointValue;
        }
    }

    private double calculateFeedforward() {
        ArmFeedforward feedforward = new ArmFeedforward(0.2, 3.9, 0.01);
        return feedforward.calculate(Math.toRadians(getDegrees()), 1);
    }

    public double getPidOutput() {
        double speed = pid.calculate(getDegrees(), setpoint) + calculateFeedforward();
        if (setpoint + 10 < getDegrees()) {
            return downSpeed;
        }
        if (speed >= maxPIDSpeed) {
            return maxPIDSpeed;
        }
        return speed;
    }

    public double getDegrees() {
        return (absoluteEncoder.getPosition() - 0.5) * -1.0 * 360.0;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        if (usingPID) {
            motor.set(getPidOutput());
        } else {
            if (getDegrees() >= maxAngle) motorOutput = 0.0;
            if (getDegrees() <= minAngle) motorOutput = 0.0;
            if (settingMinLevel) motorOutput = minPowerAtLevel;
            motor.set(motorOutput);
        }
    }
}