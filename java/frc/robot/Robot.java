/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Pneumatics imports
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Color sensor imports
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

// John's imports
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

//Falcon imports
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // Color sensor declarations
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // John's declarations
  private static final int deviceIDLauncherOne = 7;
  private static final int deviceIDLauncherTwo = 8;
  private static final int deviceIDTraveler = 9;
  private CANSparkMax m_motorLauncherOne;
  private CANSparkMax m_motorLauncherTwo;
  private CANSparkMax m_motorTraveler;
  private Joystick m_driverStick;
  private CANPIDController m_pidControllerOne;
  private CANPIDController m_pidControllerTwo;
  private CANEncoder m_encoderOne;
  private CANEncoder m_encoderTwo;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, kSP;
  public double speed7, speed8;

  //Falcon declarations
  private final WPI_TalonFX m_driveOne = new WPI_TalonFX(1);
  private final WPI_TalonFX m_driveTwo = new WPI_TalonFX(2);
  private final WPI_TalonFX m_driveThree = new WPI_TalonFX(3);
  private final WPI_TalonFX m_driveFour = new WPI_TalonFX(4);

  private final Compressor CompressorOne = new Compressor();
  private final Solenoid DriveShift = new Solenoid(0);

  private boolean bAButtonOld;
  private boolean bDriveGear;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    CompressorOne.setClosedLoopControl(true);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // === John's code ===

    // Init motor Launcher one motor
    m_motorLauncherOne = new CANSparkMax(deviceIDLauncherOne, MotorType.kBrushless);
    m_motorLauncherOne.restoreFactoryDefaults();
    // Check coast mode
    if (m_motorLauncherOne.setIdleMode(IdleMode.kCoast) != CANError.kOk) {
      // SmartDashboard.putString("Idle Mode One", "Error");
    }
    // Set coast mode
    if (m_motorLauncherOne.getIdleMode() == IdleMode.kCoast) {
      // SmartDashboard.putString("Idle Mode One", "Coast");
    } else {
      // SmartDashboard.putString("Idle Mode One", "Brake");
    }
    // Set ramp rate
    if (m_motorLauncherOne.setOpenLoopRampRate(0) != CANError.kOk) {
      // SmartDashboard.putString("Ramp Rate One", "Error");
    }
    // SmartDashboard.putNumber("Ramp Rate One",
    // m_motorLauncherOne.getOpenLoopRampRate());

    speed7 = 3000;
    SmartDashboard.putNumber("speed 7", speed7);
    speed8 = 2500;
    SmartDashboard.putNumber("speed 8", speed8);

    // For set up velocity control
    m_pidControllerOne = m_motorLauncherOne.getPIDController();
    m_encoderOne = m_motorLauncherOne.getEncoder();

    kP = 0.0003;
    kI = 0.0;
    kD = 0.0006;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
    kSP = 3000;

    m_pidControllerOne.setP(kP);
    m_pidControllerOne.setI(kI);
    m_pidControllerOne.setD(kD);
    m_pidControllerOne.setIZone(kIz);
    m_pidControllerOne.setFF(kFF);
    m_pidControllerOne.setOutputRange(kMinOutput, kMaxOutput);

    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Point", kSP);

    // Init motor Launcher two motor
    m_motorLauncherTwo = new CANSparkMax(deviceIDLauncherTwo, MotorType.kBrushless);
    m_motorLauncherTwo.restoreFactoryDefaults();
    // Check coast mode
    if (m_motorLauncherTwo.setIdleMode(IdleMode.kCoast) != CANError.kOk) {
      // SmartDashboard.putString("Idle Mode Two", "Error");
    }
    // Set coast mode
    if (m_motorLauncherTwo.getIdleMode() == IdleMode.kCoast) {
      // SmartDashboard.putString("Idle Mode Two", "Coast");
    } else {
      // SmartDashboard.putString("Idle Mode Two", "Brake");
    }
    // Set ramp rate
    if (m_motorLauncherTwo.setOpenLoopRampRate(0) != CANError.kOk) {
      // SmartDashboard.putString("Ramp Rate Two", "Error");
    }
    // SmartDashboard.putNumber("Ramp Rate Two",
    // m_motorLauncherTwo.getOpenLoopRampRate());

    // For set up velocity control
    m_pidControllerTwo = m_motorLauncherTwo.getPIDController();
    m_encoderTwo = m_motorLauncherTwo.getEncoder();

    m_pidControllerTwo.setP(kP);
    m_pidControllerTwo.setI(kI);
    m_pidControllerTwo.setD(kD);
    m_pidControllerTwo.setIZone(kIz);
    m_pidControllerTwo.setFF(kFF);
    m_pidControllerTwo.setOutputRange(kMinOutput, kMaxOutput);

    // Configure joystick
    m_driverStick = new Joystick(0);


    m_motorTraveler = new CANSparkMax(deviceIDTraveler, MotorType.kBrushless);

    // === END John's code === //

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // === Color sensor ===
    // Color detectedColor = m_colorSensor.getColor();
    // double IR = m_colorSensor.getIR();
    // int proximity = m_colorSensor.getProximity();

    // double redValue = detectedColor.red;
    // double greenValue = detectedColor.green;
    // double blueValue = detectedColor.blue;

    // double dA = Math.pow(redValue - 0.63, 2);
    // double dB = Math.pow(greenValue - 0.29, 2);
    // double dC = Math.pow(blueValue - 0.07, 2);
    // double pR = 1.0 - dA - dB - dC;

    // dA = Math.pow(redValue - 0.4, 2);
    // dB = Math.pow(greenValue - 0.5, 2);
    // dC = Math.pow(blueValue - 0.1, 2);
    // double pY = 1.0 - dA - dB - dC;

    // dA = Math.pow(redValue - 0.2, 2);
    // dB = Math.pow(greenValue - 0.6, 2);
    // dC = Math.pow(blueValue - 0.22, 2);
    // double pG = 1.0 - dA - dB - dC;

    // dA = Math.pow(redValue - 0.17, 2);
    // dB = Math.pow(greenValue - 0.45, 2);
    // dC = Math.pow(blueValue - 0.38, 2);
    // double pB = 1.0 - dA - dB - dC;

    // String currentColor = "";
    // if (pR >= 0.99) {
    // currentColor = "Red";
    // }
    // else if (pY >= 0.99) {
    // currentColor = "Yellow";
    // }
    // else if (pG >= 0.99) {
    // currentColor = "Green";
    // }
    // else if (pB >= 0.99) {
    // currentColor = "Blue";
    // }

    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("Proximity", proximity);
    // SmartDashboard.putNumber("IR", IR);
    // SmartDashboard.putNumber("pR", pR);
    // SmartDashboard.putNumber("pY", pY);
    // SmartDashboard.putNumber("pG", pG);
    // SmartDashboard.putNumber("pB", pB);
    // SmartDashboard.putString("color", currentColor);
    // === END Color sensor ===

    // === John's new code === //

    // // Set motors one and two the same as the y axis
    // m_motorLauncherOne.set(m_driverStick.getY());
    // m_motorLauncherTwo.set(m_driverStick.getY());

    final double speed7hmi = SmartDashboard.getNumber("speed 7", 0);
    final double speed8hmi = SmartDashboard.getNumber("speed 8", 0);

    if (speed7 != speed7hmi) {
      speed7 = speed7hmi;
    }
    if (speed8 != speed8hmi) {
      speed8 = speed8hmi;
    }

    // Set motors one and two to fixed values
    m_motorLauncherOne.set(-speed7 / 5700.0);
    m_motorLauncherTwo.set(speed8 / 5700.0);
    m_motorTraveler.set(m_driverStick.getRawAxis(0) / 4);


    // periodically read voltage, temperature, and applied output and publish to
    // SmartDashboard
    SmartDashboard.putNumber("Voltage LauncherOne", m_motorLauncherOne.getBusVoltage());
    SmartDashboard.putNumber("Temperature LauncherOne", m_motorLauncherOne.getMotorTemperature());
    SmartDashboard.putNumber("Output LauncherOne", m_motorLauncherOne.getAppliedOutput());
    SmartDashboard.putNumber("Voltage LauncherTwo", m_motorLauncherTwo.getBusVoltage());
    SmartDashboard.putNumber("Temperature LauncherTwo", m_motorLauncherTwo.getMotorTemperature());
    SmartDashboard.putNumber("Output LauncherTwo", m_motorLauncherTwo.getAppliedOutput());

    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double sp = SmartDashboard.getNumber("Set Point", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if ((p != kP)) { m_pidControllerOne.setP(p); m_pidControllerTwo.setP(p); kP =
    // p; }
    // if ((i != kI)) { m_pidControllerOne.setI(i); m_pidControllerTwo.setI(i); kI =
    // i; }
    // if ((d != kD)) { m_pidControllerOne.setD(d); m_pidControllerTwo.setD(d); kD =
    // d; }
    // if ((iz != kIz)) { m_pidControllerOne.setIZone(iz);
    // m_pidControllerTwo.setIZone(iz); kIz = iz; }
    // if ((ff != kFF)) { m_pidControllerOne.setFF(ff);
    // m_pidControllerTwo.setFF(ff); kFF = ff; }
    // if ((sp != kSP)) { kSP = sp; }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    // m_pidControllerOne.setOutputRange(min, max);
    // m_pidControllerTwo.setOutputRange(min, max);
    // kMinOutput = min; kMaxOutput = max;
    // }

    // double setPoint = m_driverStick.getY() * maxRPM;
    // double setPoint = sp;
    // m_pidControllerOne.setReference(-setPoint * 0.90, ControlType.kVelocity);
    // m_pidControllerTwo.setReference(setPoint, ControlType.kVelocity);
    // SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("LauncherOneVelocity", m_encoderOne.getVelocity());
    SmartDashboard.putNumber("LauncherTwoVelocity", m_encoderTwo.getVelocity());

    // === END John's new code === //

    // final double dSpeed = 0.2 * m_driverStick.getRawAxis(1);
    // double dTurn = 0.4 * m_driverStick.getRawAxis(4);

    // dTurn = Math.abs(dSpeed) < 0.05 ? dTurn * 0.35 : dTurn * 0.18;

    // final double dLeftMotors = -dSpeed + dTurn;
    // final double dRightMotors = dSpeed + dTurn;

    // final boolean bAButton = m_driverStick.getRawButton(1);
    // if (bAButton && !bAButtonOld) {
    //   bDriveGear = !bDriveGear;
    // }
    // bAButtonOld = bAButton;

    // DriveShift.set(bDriveGear);

    // m_driveOne.set(dLeftMotors);
    // m_driveTwo.set(dLeftMotors);
    // m_driveThree.set(dRightMotors);
    // m_driveFour.set(dRightMotors);

    m_driveOne.set(0);
    m_driveTwo.set(0);
    m_driveThree.set(0);
    m_driveFour.set(0);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
