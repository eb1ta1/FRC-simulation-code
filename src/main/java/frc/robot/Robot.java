// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleArmMovement;
// import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.IntakeFunctions;

public class Robot extends TimedRobot {
    private final Joystick m_joystick = new Joystick(0); // change to the variable from Constants.java

    // private final Arm m_arm = new Arm();
    // private final ExampleArmMovement m_arm_example = new ExampleArmMovement();
    private final IntakeFunctions intake = new IntakeFunctions();

    @Override
    public void robotInit() {
        new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        // m_arm.loadPreferences();
    }



    @Override
    public void simulationPeriodic() {
        // m_arm.simulationPeriodic();
    }
    
    double armPositionTriggerStopped = Constants.kMinAngleRads; // default value
    @Override
    public void teleopPeriodic() {
        // if (m_joystick.getTrigger()) {
        //     // lift the arm
        //     armPositionTriggerStopped = m_arm.getCurrentRadians(); // update the position value
        //     m_arm.reachSetpoint();
        // } else if (m_joystick.getTop()){
        //     // lower the arm
        //     armPositionTriggerStopped = m_arm.getCurrentRadians(); // update the position value
        //     int armMovementSpeed = -5; // -10 < value < 10
        //     m_arm.movePosition(armMovementSpeed);
        // } else {
        //     // keep the current arm position against gravity
        //     m_arm.keepCurrentPosition(armPositionTriggerStopped);
        // }
        // double positionDegree = m_arm.getCurrentRadians();
        // // debug output
        // System.out.println(Units.radiansToDegrees(positionDegree));
        // if (positionDegree < Constants.kMinAngleRads || positionDegree > Constants.kMaxAngleRads) {
        //     System.out.println("The arm position is too high or too low");
        // }

        if (m_joystick.getTrigger()) {
            intake.intake(1);
        }
    }

    @Override
    public void teleopExit() {
    }


    @Override
    public void close() {
        // m_arm.close();
        super.close();
    }

    @Override
    public void disabledInit() {
        // This just makes sure that our simulation code knows that the motor's off.
        // m_arm.stop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
