package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax rightMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(9, MotorType.kBrushless);
    public boolean isHolding = false;

    public IntakeSubsystem() {
        leftMotor.follow(rightMotor);
        leftMotor.setInverted(true);
    }

    // speed must be set between 0.0 to 1.0
    // isIntake==false means outtake
    public void moveMotors(double speed, boolean isIntake){
        double maxSpeed = 0.8;

        if (speed > maxSpeed) speed = maxSpeed;

        if (isHolding) {
            // holding
            rightMotor.set(0.2);
        } else {
            if (isIntake) {
                rightMotor.set(speed);
            } else {
                rightMotor.set(-1 * speed);
            }
        }
    }

    // simply stop
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }
}