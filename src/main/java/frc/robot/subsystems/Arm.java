package frc.robot.subsystems;

// import libralies

import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {
  // The P gain for the PID controller that drives this arm.
  private double m_armKp = Constants.kDefaultArmKp;
  public double m_armSetpointDegrees = Constants.kDefaultArmSetpointDegrees;
  // arm gearbox
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  // classes for PID controlling
  private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
  // classes for arm movement
  private final Encoder m_encoder =
          new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(0);

  // classes for simulation
  private final SingleJointedArmSim m_armSim =
          new SingleJointedArmSim(
                  m_armGearbox,
                  Constants.kArmReduction,
                  SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
                  Constants.kArmLength,
                  Constants.kMinAngleRads,
                  Constants.kMaxAngleRads,
                  true,
                  VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // constructor
  public Arm() {
    m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);

    Preferences.initDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    Preferences.initDouble(Constants.kArmPKey, m_armKp);
  }

  public void simulationPeriodic() {
    // set "inputs"
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // update
    m_armSim.update(0.020);

    //
    m_encoderSim.setDistance(m_armSim.getAngleRads());

    //
    RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  }

  // Load setpoint and kP from preferences.
  public void loadPreferences() {
    m_armSetpointDegrees = Preferences.getDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
      m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
      m_controller.setP(m_armKp);
    }
  }

  public void reachSetpoint() {
    var pidOutput =
            m_controller.calculate(
                    m_encoder.getDistance(), Units.degreesToRadians(m_armSetpointDegrees));
    m_motor.setVoltage(pidOutput);
  }

  public void movePosition(int speed) {
    if (speed < -10 || speed > 10) {
      System.out.println("Not proper speed value has set.");
    } else {
      var pidOutput =
              m_controller.calculate(
                      m_encoder.getDistance(),
                      m_encoder.getDistance() + Units.degreesToRadians(speed));
      m_motor.setVoltage(pidOutput);
    }
  }

  public void keepCurrentPosition(double positionTriggerStopped) {
    var pidOutput =
            m_controller.calculate(
                    m_encoder.getDistance(),
                    positionTriggerStopped);
    m_motor.setVoltage(pidOutput);
  }

  public double getCurrentRadians() {
    return m_encoder.getDistance();
  }

  public void stop() {
    m_motor.set(0.0);
  }

  public void  close() {
    m_motor.close();
    m_encoder.close();
    m_controller.close();
  }
}
