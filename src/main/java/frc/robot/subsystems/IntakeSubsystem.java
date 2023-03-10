package frc.robot.subsystems;

import org.opencv.core.MatOfFloat6;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    public final CANSparkMax rightMotor = new CANSparkMax(9, MotorType.kBrushless);
    public final CANSparkMax leftMotor = new CANSparkMax(8, MotorType.kBrushless);

    public void IntakeSubsystem() {
        rightMotor.follow(leftMotor, true);
        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();
    };

    // speed must be set between 0.0 to 1.0
    // isIntake==false means outtake


    public void intake() {
        // rightMotor.setInverted(true);
        // leftMotor.setInverted(true);

            rightMotor.set(-.5);
            leftMotor.set(.5);
        }
    
    public void outtake() {
        // rightMotor.setInverted(true);
      rightMotor.set(.7);
      leftMotor.set(-.7);
    }
    
    // simply stop
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    public void slowly() {
        rightMotor.set(-0.1);
        leftMotor.set(0.1);
    }

    public void idle() {
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
}