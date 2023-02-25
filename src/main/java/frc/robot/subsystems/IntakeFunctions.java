package frc.robot.subsystems;

// import libralies

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeFunctions {
    private final CANSparkMax left_motor = new CANSparkMax(1, MotorType.kBrushless); // NEO 550 brushless motor    
    private final CANSparkMax right_motor = new CANSparkMax(2, MotorType.kBrushless); // NEO 550 brushless motor    

    public void intake(double voltage) {
        left_motor.setVoltage(voltage);
        right_motor.setVoltage(voltage);
    }
    public void outtake(double voltage) {

    }
}
