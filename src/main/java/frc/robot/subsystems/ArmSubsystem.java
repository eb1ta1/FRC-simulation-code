package frc.robot.subsystems;

// import java.lang.Math;
// import java.time.chrono.IsoEra;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless); // read parameters from Constants.java
    private final AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private final double minAngle = 0;
    private final double maxAngle = 75;
    private final double kp = 0.5;
    private final double ki = 0.0;
    private final double kd = 0.1;
    // private final double angleOffset = 0;
    private final PIDController pid = new PIDController(kp, ki, kd);
    private double setpoint = minAngle;
    private final double setpointIncrementer = 1;
    private double motorOutput = 0.0;
    private final double maxPIDSpeed = 0.7;
    private double downSpeed = -0.3;

    // settings
    private final boolean usingPID = true;

    public ArmSubsystem() {
        absoluteEncoder.setZeroOffset(0);
        pid.setTolerance(5.0);
        motor.setInverted(false);
        // setSetpoint(minAngle);
    }

    public void raiseDebug() {
        if (usingPID) {
            motor.set(0.6);
        }
    }

    public void lowerDebug() {
        if (usingPID) {
            motor.set(-0.6);
        }
    }

    public void raise() {
        // boolean isRaise = true;
        if (usingPID) {
            setSetpoint(setpoint + setpointIncrementer);
        } else {
            motorOutput = 0.2;
        }
    }

    public void lower() {
        // boolean isLower = true;
        if (usingPID) {
            setSetpoint(setpoint - setpointIncrementer);
        } else {
            motorOutput = -0.2;
        }
    }

    // simply stop
    public void stop() {
        motorOutput = 0.0;
        motor.set(motorOutput);
    }

    public void idle() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
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

    // private double calculateFeedforward(double velocity) {    
    //     double velocity = absoluteEncoder.getVelocity();
    //     ArmFeedforward feedforward = new ArmFeedforward(0.2, 3.9, 0.01);
    //     return feedforward.calculate(Math.toRadians(getDegrees()), velocity);
    // }

    public double getPidOutput() {
        double speed = pid.calculate(getDegrees(), setpoint); // + calculateFeedforward(1);
        if (setpoint + 10 < getDegrees()) {
            return downSpeed;
        }
        if (speed >= maxPIDSpeed) {
            return maxPIDSpeed;
        }
        return speed;
    }

    public double getDegrees() {
        // double EncoderDistanceConversionFactor = (2.0 * Math.PI) / (double) 42; 
        // absoluteEncoder.setPositionConversionFactor(EncoderDistanceConversionFactor);
        double degree = (absoluteEncoder.getPosition() - 0.5) * -360.0;
        SmartDashboard.putNumber("Encoder Position", degree);
        System.out.println(degree);
        return degree;
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
        }
    }
}