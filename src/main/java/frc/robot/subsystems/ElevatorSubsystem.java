package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(7, MotorType.kBrushless);
    public RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    public RelativeEncoder rightRelativeEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    public ElevatorSubsystem() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.follow(rightMotor, true);
    }

    public void elevatorUp(double upSpeed) {
        double speed = limitSpeed(upSpeed);
        rightMotor.set(speed);
    }

    public void elevatorDown(double downSpeed) {
        double speed = limitSpeed(downSpeed);
        rightMotor.set(speed);
    }

    public void elevatorStop() {
        rightMotor.set(0);
    }

    public double limitSpeed(double speed) {
        // TODO: Read values from Constants.java
        double maxSpeed = 0.75; 
        double minSpeed = -0.5;

        if (speed > maxSpeed) {
            speed = maxSpeed;
        } else if (speed < minSpeed) {
            speed = minSpeed;
        }
        return speed;
    }

    public void periodic() {
        double rightResult = rightRelativeEncoder.getPosition();// * 360 % 360; // * (2.0 * Math.PI / 100.0) + Math.toRadians(-90);
        double leftResult = rightRelativeEncoder.getPosition();
        System.out.println("Relative " + (rightResult));
        System.out.println("Relative " + (leftResult));
        // System.out.println("Absolute " + absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Relative Encoder right", rightResult);
        SmartDashboard.putNumber("Relative Encoder left", leftResult);
    }
}
