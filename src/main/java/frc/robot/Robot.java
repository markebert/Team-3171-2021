/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

// Rev Imports
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

// Team 3171 Imports
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.auton.HardcodedAutons;
import frc.team3171.controllers.Shooter;
import frc.team3171.controllers.LightingController.Pattern;
import static frc.team3171.HelperFunctions.Deadzone_With_Map;
import static frc.team3171.HelperFunctions.Within_Percent_Error;
import frc.team3171.sensors.BNO055;
import frc.team3171.sensors.GyroPIDController;
import frc.team3171.sensors.Limelight;
import frc.team3171.drive.TalonFXMotorGroup;
import frc.team3171.drive.TractionDrive;
import frc.team3171.controllers.LightingController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue;
  private AutonRecorderData playbackData;
  private volatile double autonStartTime;
  private volatile boolean saveNewAuton;

  // Auton Mode Constants
  private static final String kDefaultAuton = "Disabled";
  private static final String kHardcodedAuton = "Hardcoded Auton";
  private static final String kRecordAutonOne = "Record BarrelRacing 1";
  private static final String kPlaybackAutonOne = "Playback BarrelRacing 1";
  private static final String kRecordAutonTwo = "Record BouncePath 3";
  private static final String kPlaybackAutonTwo = "Playback BouncePath 3";
  private static final String kRecordAutonThree = "Record BouncePath 2";
  private static final String kPlaybackAutonThree = "Playback BouncePath 2";
  private static final String kRecordAutonFour = "Record BouncePath 1";
  private static final String kPlaybackAutonFour = "Playback BouncePath 1";
  private static final String kRecordAutonFive = "Record BarrelRacing 2";
  private static final String kPlaybackAutonFive = "Playback BarrelRacing 2";

  private static final String kRecordAutonSix = "Record Slalom 2";
  private static final String kPlaybackAutonSix = "Playback Slalom 2";
  private static final String kRecordAutonSeven = "Record Auton 7";
  private static final String kPlaybackAutonSeven = "Playback Auton 7";
  private static final String kRecordAutonEight = "Record Auton 8";
  private static final String kPlaybackAutonEight = "Playback Auton 8";
  private static final String kRecordAutonNine = "Record Auton 9";
  private static final String kPlaybackAutonNine = "Playback Auton 9";
  private static final String kRecordAutonTen = "Record Auton 10";
  private static final String kPlaybackAutonTen = "Playback Auton 10";

  private static final String kRecordAutonEleven = "Record NewAuton 11";
  private static final String kPlaybackAutonEleven = "Playback NewAuton 11";
  private static final String kRecordAutonTwelve = "Record NewAuton 12";
  private static final String kPlaybackAutonTwelve = "Playback NewAuton 12";
  private static final String kRecordAutonThirteen = "Record NewAuton 13";
  private static final String kPlaybackAutonThirteen = "Playback NewAuton 13";
  private static final String kRecordAutonFourteen = "Record NewAuton 14";
  private static final String kPlaybackAutonFourteen = "Playback NewAuton 14";
  private static final String kRecordAutonFifteen = "Record NewAuton 15";
  private static final String kPlaybackAutonFifteen = "Playback NewAuton 15";

  private static final String kRecordAutonSixteen = "Record NewAuton 16";
  private static final String kPlaybackAutonSixteen = "Playback NewAuton 16";
  private static final String kRecordAutonSeventeen = "Record NewAuton 17";
  private static final String kPlaybackAutonSeventeen = "Playback NewAuton 17";
  private static final String kRecordAutonEighteen = "Record NewAuton 18";
  private static final String kPlaybackAutonEighteen = "Playback NewAuton 18";
  private static final String kRecordAutonNineteen = "Record NewAuton 19";
  private static final String kPlaybackAutonNineteen = "Playback NewAuton 19";
  private static final String kRecordAutonTwenty = "Record NewAuton 20";
  private static final String kPlaybackAutonTwenty = "Playback NewAuton 20";

  private static final String kDefaultObstacleCourseMode = "Disabled";
  private static final String kObstacleCourseMode = "Enabled";

  // Selected Auton String
  private String selectedAutonMode;

  private String selectedObstacleMode;

  // Auton Chooser
  private SendableChooser<String> autonChooser;

  private SendableChooser<String> obstacleModeChooser;

  // Joysticks
  private Joystick leftStick, rightStick, opLeft, opRight;

  // Drive Controller
  private TalonFXMotorGroup leftMotorGroup, rightMotorGroup;
  private TractionDrive driveController;
  private volatile boolean quickTurnEdgeTrigger;

  // Shooter Controller
  private Shooter shooterController;
  private volatile boolean ballpickupEdgeTrigger, reverseFeederEdgeTrigger;

  // BNO055 9-DOF IMU
  private BNO055 gyro;
  private volatile boolean gyroErrorEdgeTrigger;

  // Gyro PID Controller
  private GyroPIDController gyroPIDController;

  // Limelight Network Table
  private Limelight limelight;

  // LimelightPID Controller
  private PIDController limeLightPIDController;
  private volatile boolean targetLockEdgeTrigger;

  // Distance Sensor
  private Rev2mDistanceSensor distanceSensor;

  // Lighting Controller
  private LightingController lightController;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */


  @Override
  public void robotInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new ConcurrentLinkedQueue<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Modes init
    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption(kDefaultAuton, kDefaultAuton);
    autonChooser.addOption(kHardcodedAuton, kHardcodedAuton);
    autonChooser.addOption(kRecordAutonOne, kRecordAutonOne);
    autonChooser.addOption(kPlaybackAutonOne, kPlaybackAutonOne);
    autonChooser.addOption(kRecordAutonTwo, kRecordAutonTwo);
    autonChooser.addOption(kPlaybackAutonTwo, kPlaybackAutonTwo);
    autonChooser.addOption(kRecordAutonThree, kRecordAutonThree);
    autonChooser.addOption(kPlaybackAutonThree, kPlaybackAutonThree);
    autonChooser.addOption(kRecordAutonFour, kRecordAutonFour);
    autonChooser.addOption(kPlaybackAutonFour, kPlaybackAutonFour);
    autonChooser.addOption(kRecordAutonFive, kRecordAutonFive);
    autonChooser.addOption(kPlaybackAutonFive, kPlaybackAutonFive);

    autonChooser.addOption(kPlaybackAutonSix, kPlaybackAutonSix);
    autonChooser.addOption(kRecordAutonSix, kRecordAutonSix);
    autonChooser.addOption(kPlaybackAutonSeven, kPlaybackAutonSeven);
    autonChooser.addOption(kRecordAutonSeven, kRecordAutonSeven);
    autonChooser.addOption(kPlaybackAutonEight, kPlaybackAutonEight);
    autonChooser.addOption(kRecordAutonEight, kRecordAutonEight);
    autonChooser.addOption(kPlaybackAutonNine, kPlaybackAutonNine);
    autonChooser.addOption(kRecordAutonNine, kRecordAutonNine);
    autonChooser.addOption(kRecordAutonTen, kRecordAutonTen);
    autonChooser.addOption(kPlaybackAutonTen, kPlaybackAutonTen);

    autonChooser.addOption(kPlaybackAutonEleven, kPlaybackAutonEleven);
    autonChooser.addOption(kRecordAutonEleven, kRecordAutonEleven);
    autonChooser.addOption(kPlaybackAutonTwelve, kPlaybackAutonTwelve);
    autonChooser.addOption(kRecordAutonTwelve, kRecordAutonTwelve);
    autonChooser.addOption(kPlaybackAutonThirteen, kPlaybackAutonThirteen);
    autonChooser.addOption(kRecordAutonThirteen, kRecordAutonThirteen);
    autonChooser.addOption(kPlaybackAutonFourteen, kPlaybackAutonFourteen);
    autonChooser.addOption(kRecordAutonFourteen, kRecordAutonFourteen);
    autonChooser.addOption(kPlaybackAutonFifteen, kPlaybackAutonFifteen);
    autonChooser.addOption(kRecordAutonFifteen, kRecordAutonFifteen);

    autonChooser.addOption(kPlaybackAutonSixteen, kPlaybackAutonSixteen);
    autonChooser.addOption(kRecordAutonSixteen, kRecordAutonSixteen);
    autonChooser.addOption(kPlaybackAutonSeventeen, kPlaybackAutonSeventeen);
    autonChooser.addOption(kRecordAutonSeventeen, kRecordAutonSeventeen);
    autonChooser.addOption(kPlaybackAutonEighteen, kPlaybackAutonEighteen);
    autonChooser.addOption(kRecordAutonEighteen, kRecordAutonEighteen);
    autonChooser.addOption(kPlaybackAutonNineteen, kPlaybackAutonNineteen);
    autonChooser.addOption(kRecordAutonNineteen, kRecordAutonNineteen);
    autonChooser.addOption(kPlaybackAutonTwenty, kPlaybackAutonTwenty);
    autonChooser.addOption(kRecordAutonTwenty, kRecordAutonTwenty);

    SmartDashboard.putData("Auton Modes:", autonChooser);

    // Obstacle Course Mode
    obstacleModeChooser = new SendableChooser<>();
    obstacleModeChooser.setDefaultOption(kDefaultObstacleCourseMode, kDefaultObstacleCourseMode);
    obstacleModeChooser.addOption(kObstacleCourseMode, kObstacleCourseMode);
    SmartDashboard.putData("Obstacle Course Mode:", obstacleModeChooser);

    // Joystick init
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    opLeft = new Joystick(2);
    opRight = new Joystick(3);

    // Drive, Shooter and Climber Controller inits
    try {
      leftMotorGroup = new TalonFXMotorGroup(leftDriveCANIDArray);
      rightMotorGroup = new TalonFXMotorGroup(rightDriveCANIDArray);
      driveController = new TractionDrive(leftMotorGroup, rightMotorGroup);

      shooterController = new Shooter();
    } catch (Exception e) {
      System.err.println(e.getMessage());
    }

    // Edge Trigger init
    ballpickupEdgeTrigger = false;
    reverseFeederEdgeTrigger = false;
    quickTurnEdgeTrigger = false;
    gyroErrorEdgeTrigger = false;
    targetLockEdgeTrigger = false;

    // Gyro init
    gyro = new BNO055(SerialPort.Port.kUSB1);

    // Gyro PID Controller init
    gyroPIDController = new GyroPIDController(gyro, gyro_kP, gyro_kI, gyro_kD, -1.0, 1.0);
    gyroPIDController.start();
    gyroPIDController.disablePID();

    // Limelight init
    limelight = new Limelight();
    limelight.turnLightOff();

    // Limelight PID Controller init
    limeLightPIDController = new PIDController(limelight_kP, limelight_kI, limelight_kD);
    limeLightPIDController.enableContinuousInput(-180, 180);

    // Distance Sensor init
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kDefault);
    distanceSensor.setEnabled(true);
    distanceSensor.setAutomaticMode(true);

    // Lighting Controller init
    lightController = new LightingController(0, 113);
    lightController.reset();

    // Camera Server for climber camera
    final UsbCamera elevatorCamera = CameraServer.getInstance().startAutomaticCapture();
    elevatorCamera.setResolution(80, 60);
    elevatorCamera.setFPS(30);

    SmartDashboard.putNumber("roboInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    // Gyro Values
    SmartDashboard.putNumber("Gyro X", gyro.getX());
    SmartDashboard.putBoolean("Gyro Error:", gyro.getPossibleError());

    // Print the gyro PID values
    SmartDashboard.putNumber("Gyro PID Value:", gyroPIDController.getPIDValue());
    SmartDashboard.putNumber("Gyro Setpoint:", gyroPIDController.getSensorLockValue());

    // Limelight data
    SmartDashboard.putBoolean("Has Targets:", limelight.hasTarget());
    SmartDashboard.putNumber("Target Horizontal Offset:", limelight.getTargetHorizontalOffset());

    // Print the distance sensor values
    SmartDashboard.putNumber("Distance Sensor:", distanceSensor.getRange());

    SmartDashboard.putNumber("robotPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called once at the beginning of autonomous.
   */
  @Override
  public void autonomousInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Reset Edge Triggers
    ballpickupEdgeTrigger = false;
    quickTurnEdgeTrigger = false;
    gyroErrorEdgeTrigger = false;
    targetLockEdgeTrigger = false;

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Update Auton Selected Mode and load the auton
    selectedAutonMode = autonChooser.getSelected();
    switch (selectedAutonMode) {
      case kPlaybackAutonOne:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonOne);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonTwo:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonTwo);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonThree:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonThree);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonFour:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonFour);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonFive:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonFive);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonSix:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonSix);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonSeven:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonSeven);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonEight:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonEight);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonNine:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonNine);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonTen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonTen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonEleven:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonEleven);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonTwelve:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonTwelve);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonThirteen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonThirteen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonFourteen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonFourteen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonFifteen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonFifteen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonSixteen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonSixteen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonSeventeen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonSeventeen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonEighteen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonEighteen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonNineteen:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonNineteen);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kPlaybackAutonTwenty:
        AutonRecorder.loadFromFile(autonPlaybackQueue, kPlaybackAutonTwenty);
        playbackData = autonPlaybackQueue.poll();
        break;
      case kHardcodedAuton:
        HardcodedAutons.Auton_Init();
      default:
        playbackData = null;
        break;
    }

    // Reset the Limelight PID Controller
    limeLightPIDController.reset();
    limelight.turnLightOn();

    // Reset the gyro
    gyro.reset();

    // Resets and enables the Gyro PID Controller
    selectedObstacleMode = obstacleModeChooser.getSelected();
    switch (selectedObstacleMode) {
      case kObstacleCourseMode:
        gyroPIDController.disablePID();
        break;
      case kDefaultObstacleCourseMode:
      default:
        gyroPIDController.enablePID();
        break;
    }
    gyroPIDController.updateSensorLockValue();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("autonomousInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    switch (selectedAutonMode) {
      case kPlaybackAutonOne:
      case kPlaybackAutonTwo:
      case kPlaybackAutonThree:
      case kPlaybackAutonFour:
      case kPlaybackAutonFive:
      case kPlaybackAutonSix:
      case kPlaybackAutonSeven:
      case kPlaybackAutonEight:
      case kPlaybackAutonNine:
      case kPlaybackAutonTen:
      case kPlaybackAutonEleven:
      case kPlaybackAutonTwelve:
      case kPlaybackAutonThirteen:
      case kPlaybackAutonFourteen:
      case kPlaybackAutonFifteen:
      case kPlaybackAutonSixteen:
      case kPlaybackAutonSeventeen:
      case kPlaybackAutonEighteen:
      case kPlaybackAutonNineteen:
      case kPlaybackAutonTwenty:
        // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the latest joystick values and calculate their deadzones
          final double leftStickY = playbackData.getLeftY();
          final double rightStickX = playbackData.getRightX();

          // Get the latest joystick button values
          final boolean button_Pickup = playbackData.getPickup();
          final boolean button_Shooter = playbackData.getShooter();
          final boolean zone1 = playbackData.getShooter();
          final boolean zone2 = playbackData.getShooter();
          final boolean zone3 = playbackData.getShooter();
          final boolean zone4 = playbackData.getShooter();
          final boolean button_TargetLock = playbackData.getTargetLock();
          final boolean button_QuickTurn = playbackData.getQuickTurn();

          // Prevents jumping if the gyro is reconnected after being disconnected
          final boolean gyroError = gyro.getPossibleError();
          if (!gyroError && gyroErrorEdgeTrigger) {
            gyroPIDController.updateSensorLockValue();
          }
          gyroErrorEdgeTrigger = gyroError;

          // Checks if obstacle course mode
          switch (selectedObstacleMode) {
            case kObstacleCourseMode:
              // Drive Controls
              driveController.mecanumTraction(-leftStickY, rightStickX);
              break;
            case kDefaultObstacleCourseMode:
            default:
              // Drive Controls
              if (button_TargetLock && limelight.hasTarget() && !targetLockEdgeTrigger) {
                limeLightPIDController.reset();
              } else if (button_TargetLock && limelight.hasTarget()) {
                // Check to see if the robot has any valid targets and set the gyro lock
                driveController.mecanumTraction(-leftStickY,
                    limeLightPIDController.calculate(-limelight.getTargetHorizontalOffset(), 0));
              } else {
                if (button_QuickTurn && !quickTurnEdgeTrigger && !gyro.getPossibleError()) {
                  // Gyro turn the robot 180 degrees
                  gyroPIDController.updateSensorLockValue(gyro.getAsDouble() + 180);
                } else if (button_QuickTurn && !gyro.getPossibleError()) {
                  // Gyro Lock the robot
                  driveController.mecanumTraction(-leftStickY, gyroPIDController.getPIDValue());
                } else if (rightStickX != 0 || gyroPIDController.isDisabled() || gyro.getPossibleError()) {
                  // Manual drive control
                  gyroPIDController.updateSensorLockValue();
                  driveController.mecanumTraction(-leftStickY, rightStickX);
                } else {
                  // Gyro Lock the robot
                  driveController.mecanumTraction(-leftStickY, gyroPIDController.getPIDValue());
                }
                quickTurnEdgeTrigger = button_QuickTurn;
              }
              targetLockEdgeTrigger = button_TargetLock;
              break;
          }

          // Automated Shooter Test
          // final int lowerShooterVelocity = 2500, upperShooterVelocity = 2750;
          // final int lowerShooterVelocity = 2000, upperShooterVelocity = 3400;
          final int lowerShooterVelocity = 2000, upperShooterVelocity = 5000;
          if (button_Shooter) {
            shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
            shooterController.disengageShooterBrake();
            if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
              shooterController.setFeederSpeed(.35);
            } else {
              shooterController.setFeederSpeed(0);
            }
          } else {
            shooterController.engageShooterBrake();
            shooterController.setShooterVelocity(0);
          }
            if (zone1) {
              shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
              shooterController.disengageShooterBrake();
              if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                  && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                shooterController.setFeederSpeed(.35);
              } else {
                shooterController.setFeederSpeed(0);
              }
            } else {
              shooterController.engageShooterBrake();
              shooterController.setShooterVelocity(0);
            }
            
          if (zone2) {
                shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
                shooterController.disengageShooterBrake();
                if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                    && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                  shooterController.setFeederSpeed(.35);
                } else {
                  shooterController.setFeederSpeed(0);
                }
              } else {
                shooterController.engageShooterBrake();
                shooterController.setShooterVelocity(0);
              }
          if (zone3) {
                  shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
                  shooterController.disengageShooterBrake();
                  if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                      && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                    shooterController.setFeederSpeed(.35);
                  } else {
                    shooterController.setFeederSpeed(0);
                  }
                } else {
                  shooterController.engageShooterBrake();
                  shooterController.setShooterVelocity(0);
                }
          if (zone4) {
                    shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
                    shooterController.disengageShooterBrake();
                    if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                        && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                      shooterController.setFeederSpeed(.35);
                    } else {
                      shooterController.setFeederSpeed(0);
                    }
                  } else {
                    shooterController.engageShooterBrake();
                    shooterController.setShooterVelocity(0);
                  
                    
            // Ball Pickup Controls
            if (button_Pickup) {
              shooterController.setPickupSpeed(.8);
              final boolean pickupSensor = distanceSensor.getRange() < 125;
              if (pickupSensor && !reverseFeederEdgeTrigger) {
                shooterController.runFeeder(-.2, .25);
              } else {
                shooterController.setFeederSpeed(.3);
              }
              reverseFeederEdgeTrigger = pickupSensor;
            } else {
              shooterController.setPickupSpeed(0);
              if (ballpickupEdgeTrigger) {
                shooterController.runFeeder(-.35, .5);
              } else {
                shooterController.setFeederSpeed(0);
              }
            }
            ballpickupEdgeTrigger = button_Pickup;
          }
         
          // Lighting Controls
          if (button_Shooter || zone1 || zone2 || zone3 || zone4) {
            lightController.setPattern(Pattern.Snake_From_Center, Color.kRed, Color.kBlue);
            lightController.setDelay(.05);
          } else if (button_Pickup) {
            lightController.setPattern(Pattern.Snake_From_Center, Color.kYellow, Color.kBlack);
            lightController.setDelay(.025);
          } else {
            lightController.reset();
          }

          // Checks for new data and when to switch to it
          if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getFPGATimestamp()) {
            playbackData = autonPlaybackQueue.poll();
          }
        }else {
          disabledInit();
          selectedAutonMode = kDefaultAuton;
        }
        break;
      case kHardcodedAuton:
        // HardcodedAutons.Auton_Center(driveController, gyro, gyroPIDController,
        // shooterController);
        break;
      case kRecordAutonOne:
      case kRecordAutonTwo:
      case kRecordAutonThree:
      case kRecordAutonFour:
      case kRecordAutonFive:
      case kRecordAutonSix:
      case kRecordAutonSeven:
      case kRecordAutonEight:
      case kRecordAutonNine:
      case kRecordAutonTen:
      case kRecordAutonEleven:
      case kRecordAutonTwelve:
      case kRecordAutonThirteen:
      case kRecordAutonFourteen:
      case kRecordAutonFifteen:
      case kDefaultAuton:

      default:
        // Do Nothing
        break;
    }

    SmartDashboard.putNumber("autonomousPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called once at the beginning of operator control.
   */
  @Override
  public void teleopInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Reset all of the Edge Triggers
    ballpickupEdgeTrigger = false;
    reverseFeederEdgeTrigger = false;
    quickTurnEdgeTrigger = false;
    gyroErrorEdgeTrigger = false;
    targetLockEdgeTrigger = false;

    // Reset Drive Direction
    driveController.setDriveDirectionFlipped(false);

    // Update Auton Selected Mode and reset the data recorder
    selectedAutonMode = autonChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = false;

    // Reset the Limelight PID Controller
    limeLightPIDController.reset();
    limelight.turnLightOn();

    // Resets and enables the Gyro PID Controller
    selectedObstacleMode = obstacleModeChooser.getSelected();
    switch (selectedObstacleMode) {
      case kObstacleCourseMode:
        gyroPIDController.disablePID();
        break;
      case kDefaultObstacleCourseMode:
      default:
        gyroPIDController.enablePID();
        break;
    }
    gyroPIDController.updateSensorLockValue();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("teleopInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    // Get the latest joystick values and calculate their deadzones
    final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, leftStick.getY());
    final double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, rightStick.getX());

    // Get the latest joystick button values
    final boolean button_Pickup = leftStick.getTrigger();
    final boolean button_ReversePickup = leftStick.getRawButton(3);
    final boolean button_ReverseFeeder = leftStick.getRawButton(4);

    final boolean button_Shooter = rightStick.getTrigger();
    final boolean button_TargetLock = rightStick.getRawButton(2);
    final boolean button_QuickTurn = rightStick.getRawButton(3);
    final boolean button_ShooterYEET = rightStick.getRawButton(4);

    final boolean zone1 = opLeft.getTrigger();
    final boolean zone2 = opLeft.getRawButton(2);
    final boolean zone3 = opLeft.getRawButton(3);
    final boolean zone4 = opLeft.getRawButton(4);

    final boolean zoneStellar = opRight.getTrigger();

    // Auton Recording
    switch (selectedAutonMode) {
      case kRecordAutonFifteen:
      case kRecordAutonFourteen:
      case kRecordAutonThirteen:
      case kRecordAutonTwelve:
      case kRecordAutonEleven:
      case kRecordAutonTen:
      case kRecordAutonNine:
      case kRecordAutonEight:
      case kRecordAutonSeven:
      case kRecordAutonSix:
      case kRecordAutonFive:
      case kRecordAutonFour:
      case kRecordAutonThree:
      case kRecordAutonTwo:
      case kRecordAutonOne:
        final double autonFPGATimestamp = Timer.getFPGATimestamp() - autonStartTime;
        saveNewAuton = true;
        // Crate new auton data packet and record the data
        final AutonRecorderData newData = new AutonRecorderData();
        newData.setFPGATimestamp(autonFPGATimestamp);
        // Checks if obstacle course mode
        switch (selectedObstacleMode) {
          case kObstacleCourseMode:
            newData.setLeftY(leftStickY * MAX_Y_DRIVE_SPEED);
            newData.setRightX(rightStickX * MAX_X_DRIVE_SPEED);
            break;
          case kDefaultObstacleCourseMode:
          default:
            newData.setLeftY(leftStickY);
            newData.setRightX(rightStickX);
            break;
        }
        newData.setShooter(button_Shooter);
        newData.setShooter(zone1);
        newData.setShooter(zone2);
        newData.setShooter(zone3);
        newData.setShooter(zone4);
        newData.setPickup(button_Pickup);
        newData.setTargetLock(button_TargetLock);
        newData.setQuickTurn(button_QuickTurn);
        // Adds the recorded data to the auton recorder, but only if the data is new
        autonRecorder.addNewData(newData);
        break;
      default:
        // Do Nothing
        break;
    }

    // Prevents jumping if the gyro is reconnected after being disconnected
    final boolean gyroError = gyro.getPossibleError();
    if (!gyroError && gyroErrorEdgeTrigger) {
      gyroPIDController.updateSensorLockValue();
    }
    gyroErrorEdgeTrigger = gyroError;

    // Checks if obstacle course mode
    switch (selectedObstacleMode) {
      case kObstacleCourseMode:
        // Drive Controls
        driveController.mecanumTraction(-leftStickY * MAX_Y_DRIVE_SPEED, rightStickX * MAX_X_DRIVE_SPEED);
        break;
      case kDefaultObstacleCourseMode:
      default:
        // Drive Controls
        if (button_TargetLock && limelight.hasTarget() && !targetLockEdgeTrigger) {
          limeLightPIDController.reset();
        } else if (button_TargetLock && limelight.hasTarget()) {
          // Check to see if the robot has any valid targets and set the gyro lock
          driveController.mecanumTraction(-leftStickY,
              limeLightPIDController.calculate(-limelight.getTargetHorizontalOffset(), 0));
        } else {
          if (button_QuickTurn && !quickTurnEdgeTrigger && !gyro.getPossibleError()) {
            // Gyro turn the robot 180 degrees
            gyroPIDController.updateSensorLockValue(gyro.getAsDouble() + 180);
          } else if (button_QuickTurn && !gyro.getPossibleError()) {
            // Gyro Lock the robot
            driveController.mecanumTraction(-leftStickY, gyroPIDController.getPIDValue());
          } else if (rightStickX != 0 || gyroPIDController.isDisabled() || gyro.getPossibleError()) {
            // Manual drive control
            gyroPIDController.updateSensorLockValue();
            driveController.mecanumTraction(-leftStickY, rightStickX);
          } else {
            // Gyro Lock the robot
            driveController.mecanumTraction(-leftStickY, gyroPIDController.getPIDValue());
          }
          quickTurnEdgeTrigger = button_QuickTurn;
        }
        targetLockEdgeTrigger = button_TargetLock;
        break;
    }

    // Automated Shooter Test
    // final int lowerShooterVelocity = 2500, upperShooterVelocity = 2750;
    // final int lowerShooterVelocity = 2000, upperShooterVelocity = 3400;
    final int lowerShooterVelocity = 2000, upperShooterVelocity = 5000;
    if (button_Shooter) {
      shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
      shooterController.disengageShooterBrake();
      if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), lowerShooterVelocity, .05)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
        shooterController.setFeederSpeed(.35);
      } else {
        shooterController.setFeederSpeed(0);
      }
  
    } else if (zone1) {
      shooterController.setShooterVelocity(1700, 4700);
      shooterController.disengageShooterBrake();
      if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), 1700, .05)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), 4700, .05)) {
        shooterController.setFeederSpeed(.25);
      } else {
        shooterController.setFeederSpeed(0);
      }

    } else if (zone2) {
      shooterController.setShooterVelocity(1400, 4500);
      shooterController.disengageShooterBrake();
      if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), 1400, .05)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), 4500, .05)) {
        shooterController.setFeederSpeed(.25);
      } else {
        shooterController.setFeederSpeed(0);
      }

    } else if (zone3) {
      shooterController.setShooterVelocity(1400, 5700);
      shooterController.disengageShooterBrake();
      if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), 1400, .05)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), 5700, .05)) {
        shooterController.setFeederSpeed(.25);
      } else {
        shooterController.setFeederSpeed(0);
      }

    } else if (zone4) {
      shooterController.setShooterVelocity(5750, 3150);
      shooterController.disengageShooterBrake();
      if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), 5750, .05)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), 3150, .05)) {
        shooterController.setFeederSpeed(.25);
      } else {
        shooterController.setFeederSpeed(0);
      }

    } else if (zoneStellar) {
      shooterController.setShooterVelocity(1300, 5500);
      shooterController.disengageShooterBrake();
      if (Within_Percent_Error(shooterController.getLowerShooterVelocity(), 1300, .05)
          && Within_Percent_Error(shooterController.getUpperShooterVelocity(), 5500, .05)) {
        shooterController.setFeederSpeed(.38);
      } else {
        shooterController.setFeederSpeed(0);
      }


    } else if (button_ShooterYEET) {
      // Shooter Yeet
      shooterController.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
      shooterController.disengageShooterBrake();
      shooterController.setFeederSpeed(1);
      shooterController.setPickupSpeed(1);
    } else {
      shooterController.engageShooterBrake();
      shooterController.setShooterVelocity(0);
      // Ball Pickup Controls
      if (button_Pickup) {
        shooterController.setPickupSpeed(.8);
        final boolean pickupSensor = distanceSensor.getRange() < 125;
        if (pickupSensor && !reverseFeederEdgeTrigger) {
          shooterController.runFeeder(-.2, .25);
        } else {
          shooterController.setFeederSpeed(.3);
        }
        reverseFeederEdgeTrigger = pickupSensor;
      } else if (button_ReversePickup) {
        shooterController.setPickupSpeed(-.8);
      } else if (button_ReverseFeeder) {
        shooterController.setFeederSpeed(-.3);
      } else {
        shooterController.setPickupSpeed(0);
        if (ballpickupEdgeTrigger) {
          shooterController.runFeeder(-.35, .5);
        } else {
          shooterController.setFeederSpeed(0);
        }
      }
      ballpickupEdgeTrigger = button_Pickup;
    }

    // Lighting Controls
    if (button_Shooter || zone1 || zone2 || zone3 || zone4 || zoneStellar) {
      lightController.setPattern(Pattern.Snake_From_Center, Color.kRed, Color.kBlue);
      lightController.setDelay(.05);
    } else if (button_Pickup) {
      lightController.setPattern(Pattern.Snake_From_Center, Color.kYellow, Color.kBlack);
      lightController.setDelay(.025);
    } else {
      lightController.reset();
    }

    SmartDashboard.putNumber("teleopPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called once at the beginning of the disabled mode.
   */
  @Override
  public void disabledInit() {
    final double startTime = Timer.getFPGATimestamp();

    // Disabled all of the robot controllers
    driveController.disable();
    shooterController.disable();

    // Resets the PID Controllers
    gyroPIDController.disablePID();
    limeLightPIDController.reset();
    limelight.turnLightOff();

    if (saveNewAuton) {
      saveNewAuton = false;
      // Once auton recording is done, save the data to a file, if there is any
      switch (selectedAutonMode) {
        case kRecordAutonOne:
          autonRecorder.saveToFile(kPlaybackAutonOne);
          break;
        case kRecordAutonTwo:
          autonRecorder.saveToFile(kPlaybackAutonTwo);
          break;
        case kRecordAutonThree:
          autonRecorder.saveToFile(kPlaybackAutonThree);
          break;
        case kRecordAutonFour:
          autonRecorder.saveToFile(kPlaybackAutonFour);
          break;
        case kRecordAutonFive:
          autonRecorder.saveToFile(kPlaybackAutonFive);
          break;
        case kRecordAutonSix:
          autonRecorder.saveToFile(kPlaybackAutonSix);
          break;
        case kRecordAutonSeven:
          autonRecorder.saveToFile(kPlaybackAutonSeven);
          break;
        case kRecordAutonEight:
          autonRecorder.saveToFile(kPlaybackAutonEight);
          break;
        case kRecordAutonNine:
          autonRecorder.saveToFile(kPlaybackAutonNine);
          break;
        case kRecordAutonTen:
          autonRecorder.saveToFile(kPlaybackAutonTen);
          break;
          case kRecordAutonEleven:
          autonRecorder.saveToFile(kPlaybackAutonEleven);
          break;
        case kRecordAutonTwelve:
          autonRecorder.saveToFile(kPlaybackAutonTwelve);
          break;
        case kRecordAutonThirteen:
          autonRecorder.saveToFile(kPlaybackAutonThirteen);
          break;
        case kRecordAutonFourteen:
          autonRecorder.saveToFile(kPlaybackAutonFourteen);
          break;
        case kRecordAutonFifteen:
          autonRecorder.saveToFile(kPlaybackAutonFifteen);
          break;
        case kRecordAutonSixteen:
          autonRecorder.saveToFile(kPlaybackAutonSixteen);
          break;
        case kRecordAutonSeventeen:
          autonRecorder.saveToFile(kPlaybackAutonSeventeen);
          break;
        case kRecordAutonEighteen:
          autonRecorder.saveToFile(kPlaybackAutonEighteen);
          break;
        case kRecordAutonNineteen:
          autonRecorder.saveToFile(kPlaybackAutonNineteen);
          break;
        case kRecordAutonTwenty:
          autonRecorder.saveToFile(kPlaybackAutonTwenty);
          break;
        default:
          // Do Nothing
          break;
      }
    }

    // Reset all of the Edge Triggers
    ballpickupEdgeTrigger = false;
    reverseFeederEdgeTrigger = false;
    quickTurnEdgeTrigger = false;
    gyroErrorEdgeTrigger = false;
    targetLockEdgeTrigger = false;

    SmartDashboard.putNumber("disabledInit:", Timer.getFPGATimestamp() - startTime);
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    final double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("disabledPeriodic:", Timer.getFPGATimestamp() - startTime);
  }

}
