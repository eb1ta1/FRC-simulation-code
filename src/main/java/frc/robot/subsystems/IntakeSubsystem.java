package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public final CANSparkMax rightMotor = new CANSparkMax(9, MotorType.kBrushless);
    public final CANSparkMax leftMotor = new CANSparkMax(8, MotorType.kBrushless);
    public final double holdingSpeed = -0.1;
    public final double maxSpeed = 0.7;
    // public final double minSpeed = -0.7;

    public IntakeSubsystem() {
        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.follow(rightMotor, true);
    }

    public void intake(double strength) {
        double motorSpinSpeed = limitSpeed(strength);
        rightMotor.set(motorSpinSpeed);
    }
    
    public void outtake(double strength) {
        double motorSpinSpeed = limitSpeed(strength);
        rightMotor.set(-motorSpinSpeed);
    }
    
    // simply stop
    public void stop() {
        rightMotor.set(0);
    }

    public void holding() {
        rightMotor.set(holdingSpeed);
    }

    public double limitSpeed(double speed) {
        // TODO: Read values from Constants.java
        double maxSpeed = 0.75; 
        // double minSpeed = -0.5;

        if (speed > maxSpeed) {
            speed = maxSpeed;
        // } else if (speed < minSpeed) {
        //     speed = minSpeed;
            }
        return maxSpeed;
        }
}
