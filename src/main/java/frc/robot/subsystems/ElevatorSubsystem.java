package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(7, MotorType.kBrushless);
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
}
