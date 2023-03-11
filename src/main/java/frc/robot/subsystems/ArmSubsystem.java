package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless); // read parameters from Constants.java
    public RelativeEncoder relativeEncoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    private final double bottomEncoderPosition = 15.95;
    private final double topEncoderPosition = -3.8; // TODO: Read the value from Constants
    private final double maxMotorSpeed = 0.8;
    private final double defaultLowerSpeed = 0.5;
    private final double kp = 0.5;
    private final double ki = 0.0;
    private final double kd = 0.1;
    private final PIDController pid = new PIDController(kp, ki, kd);
    private final DigitalInput toplimitSwitch = new DigitalInput(0); // TODO: read from Constants
    private double position = 0; // default position with stopper
    private final double setpointIncrementer = 0.2;

    public ArmSubsystem() {
        pid.setTolerance(1);
    }

    // Debug functions
    public void debugRaise(double speed) {
        motor.set(speed);
    }

    public void debugLower(double speed) {
        motor.set(-speed);
    }

    // Stop
    public void stop() {
        motor.set(0);
    }

    // Setpoint vertification
    public double positionUpdate(double position) {
        if (position >= bottomEncoderPosition) {
            return bottomEncoderPosition;
        } else if (position <= topEncoderPosition) {
            return topEncoderPosition;
        } else {
            return position;
        }
    }

    private double calculateFeedforward() {
        double velocity = relativeEncoder.getVelocity();
        ArmFeedforward feedforward = new ArmFeedforward(0.2, 3.9, 0.01);
        return feedforward.calculate(Math.toRadians(getMeasurement()), velocity);
    }

    // Get speed from PID
    public double getPidOutput() {
        double currentPosition = getMeasurement();
        double speed = pid.calculate(currentPosition, position); // + calculateFeedforward();
        if (currentPosition +1 >= topEncoderPosition) {
            // close to bottom
            speed = -defaultLowerSpeed;
        } else if (currentPosition <= position - 1) {
            speed = defaultLowerSpeed;
        }
        return speed;
    }

    // Get current degree
    public double getMeasurement() {
        return relativeEncoder.getPosition();
    }

    // Move
    public void raise() {
        position = positionUpdate(getMeasurement() - setpointIncrementer);
    }

    public void lower() {
        position = positionUpdate(getMeasurement() + setpointIncrementer);
    }

    @Override
    public void periodic() {
        motor.set(getPidOutput());
    }
}
