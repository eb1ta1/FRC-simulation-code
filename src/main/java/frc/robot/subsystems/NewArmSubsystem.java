package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
// import java.lang.Math;
// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class NewArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless); // read parameters from Constants.java
    public RelativeEncoder relativeEncoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private final double minAngle = 0;
    private final double maxAngle = 75;
    private final double maxMotorSpeed = 0.8;
    private final double defaultLowerSpeed = 0.5;
    private final double kp = 0.5;
    private final double ki = 0.0;
    private final double kd = 0.1;
    private final PIDController pid = new PIDController(kp, ki, kd);

    // default position
    private double setpoint = minAngle;
    private final double setpointIncrementer = 1;

    public NewArmSubsystem() {
        absoluteEncoder.setZeroOffset(0);
        pid.setTolerance(1);
    }

    // Debug functions
    public void debugRaise() {
        motor.set(0.6);
    }

    public void debugLower() {
        motor.set(0.6);
    }
// Idle function
    public void idleMode() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    // Stop
    public void stop() {
        motor.set(0);
    }

    // Setpoint vertification
    public double setPosition(double setpoint) {
        double vertifiedSetpoint;

        if (setpoint >= maxAngle) {
            vertifiedSetpoint = maxAngle;
        } else if (setpoint <= minAngle) {
            vertifiedSetpoint = minAngle;
        } else {
            vertifiedSetpoint = setpoint;
        }

        return vertifiedSetpoint;
    }

    // Get speed from PID
    public double getPidOutput() {
        double speed = pid.calculate(getDegrees(), setpoint);
        if (setpoint + 10 < getDegrees()) {
            speed = defaultLowerSpeed;
        }
        if (speed >= maxMotorSpeed) {
            speed = maxMotorSpeed;
        }
        return speed;
    }

    // Get current degree
    public double getDegrees() {
        double degree = absoluteEncoder.getPosition() * 360.0;
        SmartDashboard.putNumber("Encoder Position", degree);
        // System.out.println("Absolute " + absoluteEncoder.getPosition());
        System.out.println("Relative " + relativeEncoder.getPosition());
        SmartDashboard.putNumber("Relative Encoder", relativeEncoder.getPosition());
        return relativeEncoder.getPosition() * 360;
    }
    // Move
    public void raise() {
        setpoint = setPosition(getDegrees() + setpointIncrementer);
    }
    public void lower() {
        setpoint = setPosition(getDegrees() - setpointIncrementer);
    }

    @Override
    public void periodic() {
        motor.set(getPidOutput());
    }
}