// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
 WPI_TalonSRX LeftA = new WPI_TalonSRX(1);
 WPI_TalonSRX LeftB = new WPI_TalonSRX(3);
 WPI_TalonSRX RightA = new WPI_TalonSRX(2);
 WPI_TalonSRX RightB = new WPI_TalonSRX(4);
 MotorControllerGroup leftMotors = new MotorControllerGroup(LeftA, LeftB);
 MotorControllerGroup rightMotors = new MotorControllerGroup(RightA,RightB);
 DifferentialDrive diffDrive = new DifferentialDrive (leftMotors, rightMotors);

  public DriveTrain() {
    diffDrive.setMaxOutput(0.67);
    rightMotors.setInverted(true);


  }
public void arcadeDrive(double forward, double turn){
  diffDrive.arcadeDrive(forward, turn); }
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
