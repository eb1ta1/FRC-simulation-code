package frc.robot.subsystems;

// import libralies

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ExampleArmMovement {
    private final CANSparkMax m_motor = new CANSparkMax(5, MotorType.kBrushless);
    public void movePosition() {
        m_motor.setVoltage(1);
    }
}
