package frc.team3171.auton;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

// Rev Imports
import com.revrobotics.Rev2mDistanceSensor;

// Team 3171 Imports
import frc.team3171.controllers.Shooter;
import frc.team3171.drive.TractionDrive;
import frc.team3171.sensors.BNO055;
import frc.team3171.sensors.GyroPIDController;
import static frc.team3171.HelperFunctions.Within_Percent_Error;

/**
 * @author Mark Ebert
 */
public class HardcodedAutons {

    // Auton Start Time
    private static double autonStartTime = 0;

    // Edge Triggers
    private static boolean quickTurnEdgeTrigger = false, reverseFeederEdgeTrigger = false;

    // Hidden Constructor
    private HardcodedAutons() {
    }

    /**
     * Must be called during {@link TimedRobot#autonomousInit()} prior to executing
     * any of the {@linkplain HardcodedAutons} to prevent any issues from occurring.
     */
    public static void Auton_Init() {
        autonStartTime = Timer.getFPGATimestamp();
        quickTurnEdgeTrigger = false;
        reverseFeederEdgeTrigger = false;
    }

    /**
     * Auton suitable for use in the center starting position of the field.
     * 
     * @param drive         The drive controller of the robot.
     * @param gyro          The gyro used by the robot.
     * @param pidController The pid controller used by the robot for driving.
     * @param shooter       The shooter controller of the robot.
     */
    public static void Auton_Center(TractionDrive drive, BNO055 gyro, GyroPIDController pidController,
            Shooter shooter) {
        final double currentTime = Timer.getFPGATimestamp() - autonStartTime;
        if (currentTime < 4.5) {
            // Automated Shooter
            final int lowerShooterVelocity = 2000, upperShooterVelocity = 5000;
            shooter.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
            shooter.disengageShooterBrake();
            if (Within_Percent_Error(shooter.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                    && Within_Percent_Error(shooter.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                shooter.setFeederSpeed(.35);
            } else {
                shooter.setFeederSpeed(0);
            }
        } else if (currentTime < 6) {
            shooter.engageShooterBrake();
            shooter.setShooterVelocity(0);
            drive.mecanumTraction(-.3, 0);
        } else if (currentTime < 8) {
            if (!gyro.getPossibleError()) {
                if (!quickTurnEdgeTrigger) {
                    quickTurnEdgeTrigger = true;
                    pidController.enablePID();
                    pidController.updateSensorLockValue(pidController.getSensorValue() + 180);
                } else {
                    drive.mecanumTraction(0, pidController.getPIDValue());
                }
            } else {
                drive.mecanumTraction(0, 0);
            }
        } else {
            quickTurnEdgeTrigger = false;
            pidController.disablePID();
            drive.disable();
            shooter.disable();
        }
    }

    /**
     * Auton suitable for use in the left starting position of the field.
     * 
     * @param drive         The drive controller of the robot.
     * @param gyro          The gyro used by the robot.
     * @param pidController The pid controller used by the robot for driving.
     * @param shooter       The shooter controller of the robot.
     */
    public static void Auton_Left(TractionDrive drive, BNO055 gyro, GyroPIDController pidController, Shooter shooter) {
        final double currentTime = Timer.getFPGATimestamp() - autonStartTime;
        if (currentTime < 4.5) {
            // Automated Shooter
            final int lowerShooterVelocity = 2000, upperShooterVelocity = 5000;
            shooter.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
            shooter.disengageShooterBrake();
            if (Within_Percent_Error(shooter.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                    && Within_Percent_Error(shooter.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                shooter.setFeederSpeed(.35);
            } else {
                shooter.setFeederSpeed(0);
            }
        } else if (currentTime < 6) {
            shooter.engageShooterBrake();
            shooter.setShooterVelocity(0);
            drive.mecanumTraction(-.3, 0);
        } else if (currentTime < 8) {
            if (!gyro.getPossibleError()) {
                if (!quickTurnEdgeTrigger) {
                    quickTurnEdgeTrigger = true;
                    pidController.enablePID();
                    pidController.updateSensorLockValue(pidController.getSensorValue() + 180);
                } else {
                    drive.mecanumTraction(0, pidController.getPIDValue());
                }
            } else {
                drive.mecanumTraction(0, 0);
            }
        } else {
            quickTurnEdgeTrigger = false;
            pidController.disablePID();
            drive.disable();
            shooter.disable();
        }
    }

    /**
     * Auton suitable for use in the right starting position of the field.
     * 
     * @param drive         The drive controller of the robot.
     * @param gyro          The gyro used by the robot.
     * @param pidController The pid controller used by the robot for driving.
     * @param shooter       The shooter controller of the robot.
     */
    public static void Auton_Right(TractionDrive drive, BNO055 gyro, GyroPIDController pidController, Shooter shooter,
            Rev2mDistanceSensor sensor) {
        final double currentTime = Timer.getFPGATimestamp() - autonStartTime;
        if (currentTime < 4.5) {
            // Automated Shooter
            final int lowerShooterVelocity = 2000, upperShooterVelocity = 5000;
            shooter.setShooterVelocity(lowerShooterVelocity, upperShooterVelocity);
            shooter.disengageShooterBrake();
            if (Within_Percent_Error(shooter.getLowerShooterVelocity(), lowerShooterVelocity, .05)
                    && Within_Percent_Error(shooter.getUpperShooterVelocity(), upperShooterVelocity, .05)) {
                shooter.setFeederSpeed(.35);
            } else {
                shooter.setFeederSpeed(0);
            }
        } else if (currentTime < 6) {
            shooter.engageShooterBrake();
            shooter.setShooterVelocity(0);
            drive.mecanumTraction(-.3, 0);
        } else if (currentTime < 8) {
            if (!gyro.getPossibleError()) {
                if (!quickTurnEdgeTrigger) {
                    quickTurnEdgeTrigger = true;
                    pidController.enablePID();
                    pidController.updateSensorLockValue(pidController.getSensorValue() - 170);
                } else {
                    drive.mecanumTraction(0, pidController.getPIDValue());
                }
            } else {
                drive.mecanumTraction(0, 0);
            }
        } else if (currentTime < 10) {
            drive.mecanumTraction(-.3, 0);
            shooter.setPickupSpeed(.8);
            final boolean pickupSensor = sensor.getRange() < 125;
            if (pickupSensor && !reverseFeederEdgeTrigger) {
                shooter.runFeeder(-.2, .25);
            } else {
                shooter.setFeederSpeed(.3);
            }
            reverseFeederEdgeTrigger = pickupSensor;
        } else if (currentTime < 10.5) {
            drive.mecanumTraction(0, 0);
            shooter.setPickupSpeed(0);
            shooter.runFeeder(-.35, .5);
            quickTurnEdgeTrigger = false;
        } else if (currentTime < 12.5) {
            if (!gyro.getPossibleError()) {
                if (!quickTurnEdgeTrigger) {
                    quickTurnEdgeTrigger = true;
                    pidController.enablePID();
                    pidController.updateSensorLockValue(pidController.getSensorValue() - 170);
                } else {
                    drive.mecanumTraction(0, pidController.getPIDValue());
                }
            } else {
                drive.mecanumTraction(0, 0);
            }
        } else {
            quickTurnEdgeTrigger = false;
            pidController.disablePID();
            drive.disable();
            shooter.disable();
        }
    }

}