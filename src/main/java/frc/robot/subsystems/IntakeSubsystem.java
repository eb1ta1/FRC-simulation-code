package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(9, MotorType.kBrushless);
    // leftMotor.follow(rightMotor, true);

    public void intake(Boolean isInversed){
        double defaultSpeed = 0.8;
        if (isInversed) defaultSpeed = -1 * defaultSpeed;
        rightMotor.set(defaultSpeed);
        leftMotor.set(-1 * defaultSpeed);
    }
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }
}