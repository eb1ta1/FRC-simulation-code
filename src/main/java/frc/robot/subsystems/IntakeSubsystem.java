package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase{
    public CANSparkMax rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    public CANSparkMax leftMotor = new CANSparkMax(9, MotorType.kBrushless);
    // leftMotor.follow(rightMotor, true);

    public void intake(){
        rightMotor.set(0.8);
        leftMotor.set(-0.8);
    }

    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }
}