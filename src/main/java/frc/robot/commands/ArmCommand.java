package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
    private final ArmSubsystem m_arm;
    double m_speed;

    public ArmCommand(ArmSubsystem subsystem, double speed) {
        m_arm = subsystem;
        m_speed = speed;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_arm.debugRaise(0.5);
    }
}
