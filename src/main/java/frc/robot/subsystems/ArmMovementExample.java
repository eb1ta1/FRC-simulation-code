package frc.robot.subsystems;

// import libralies

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmMovementExample {
    private final CANSparkMax m_motor = new CANSparkMax(5, MotorType.kBrushless);
    public void movePosition() {
        m_motor.setVoltage(6);
    }
}
