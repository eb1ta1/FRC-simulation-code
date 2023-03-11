// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;
 
public class RobotContainer {
  public static XboxController driveController = new XboxController(1);
  // private final Joystick m_joystick = new Joystick(0);
  private final DriveTrain m_drive = new DriveTrain();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(getArcadeDriveCommand());
  }
  public Command getArcadeDriveCommand(){
    return new ArcadeDrive(m_drive, () -> -driveController.getRawAxis(1), () -> driveController.getRawAxis(4));}
  }

