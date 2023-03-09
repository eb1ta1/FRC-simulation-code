// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// e import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.ExampleArmMovement;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NewArmSubsystem;
// import edu.wpi.first.wpilibj.TimedRobot;
// import frc.robot.subsystems.IntakeFunctions;
import frc.robot.subsystems.ArmSubsystem;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import java.lang.Math;
import java.math.RoundingMode;
import java.text.DecimalFormat;



public class Robot extends TimedRobot {
    private final Joystick m_joystick = new Joystick(0); // change to the variable from Constants.java

    // private final Arm m_arm = new Arm();
    // private final ExampleArmMovement m_arm_example = new ExampleArmMovement();
    // private final IntakeFunctions intake = new IntakeFunctions();

    public final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless); // read parameters from Constants.java
    public SparkMaxAbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    public RelativeEncoder relativeEncoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    @Override
    public void robotInit() {
        // absoluteEncoder.setZeroOffset(0);
        motor.restoreFactoryDefaults();
        
    }
    
    // private final ArmSubsystem arm = new ArmSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    // private final NewArmSubsystem newArm = new NewArmSubsystem();
    double outtakeDefaultSpeed = 0.2;
    boolean isIntakeHolding = false;
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        new RobotContainer();
        double v = relativeEncoder.getPosition();
        System.out.println("Relative " + v);
        // arm.debugGetDegrees();
        // arm.debugGetDegrees();
        // // default mode (idle)
        // arm.raiseDebug();
        // intake.stop();

        // if (m_joystick.getRawButton(1)) {
        //     arm.raiseDebug();
        // } else if (m_joystick.getRawButton(2)) {
        //     // arm.lower();
        // } else if (m_joystick.getRawButton(3)) {
        //     intake.moveMotors(0.5, true);
        // } else if (m_joystick.getRawButton(4)) {
        // }
        
        motor.set(0.5);
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
        motor.set(1.0);
    }
    double armPositionTriggerStopped = Constants.kMinAngleRads; // default value
    @Override
    public void teleopPeriodic() {

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
        // arm.stop();
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
