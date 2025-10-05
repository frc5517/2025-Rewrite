package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.robot.Telemetry;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.robot.Telemetry.*;

public class IntakeShooter extends SubsystemBase {
    // This is just a basic single wheel being run at a duty cycle speed no need to use YAMS

    /**
     * The coral and algae sensors and triggers.
     */
    private final DigitalInput coralSensor = new DigitalInput(HardwareConstants.kCoralSensorID);
    private final DigitalInput algaeSensor = new DigitalInput(HardwareConstants.kAlgaeSensorID);
    private final Trigger algaeTrigger = new Trigger(() -> !algaeSensor.get());
    /**
     * The motor driving the intake shooter mechanism.
     */
    private final SparkMax motor = new SparkMax(HardwareConstants.kMotorID, SparkLowLevel.MotorType.kBrushless);
    /**
     * Subsystems used to help run the simulation.
     */
    private final SwerveSubsystem swerve;
    private final Elevator elevator;
    private final Arm arm;
    private Trigger coralTrigger = new Trigger(coralSensor::get);
    /**
     * Intake simulation to end effect the simulated game pieces.
     */
    private IntakeSimulation intakeSimulation = null;
    private boolean infiniteAmmo = false;
    /**
     * Intake Shooter Publishers and Subscriber
     */
    private final BooleanPublisher simShootPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry")
            .getBooleanTopic("Unlimited Ammo")
            .publish();
    private final BooleanSubscriber simShootSubscriber = simShootPublisher.getTopic().subscribe(false);

    private final Trigger unlimitedAmmoTrigger = new Trigger(simShootSubscriber::get);

    /**
     * Initializes the intake shooter subsystem.
     * Calls for some additional subsystems for simulation purposes.
     */
    public IntakeShooter(SwerveSubsystem drivebase, Elevator elevator, Arm arm) {
        this.swerve = drivebase;
        this.elevator = elevator;
        this.arm = arm;

        if (RobotBase.isSimulation()) {
            this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                    "Coral",
                    drivebase.getMapleDrive(),
                    Inches.of(10),
                    IntakeSimulation.IntakeSide.FRONT,
                    1
            );
            coralTrigger = new Trigger(() -> intakeSimulation.getGamePiecesAmount() > 0);
        }
        simShootPublisher.setDefault(false);
    }

    @Override
    public void periodic() {
        if (Telemetry.robotVerbosity.ordinal() >= Telemetry.RobotTelemetry.LOW.ordinal()) {
            coralSensorPublisher.set(coralTrigger.getAsBoolean());
            algaeSensorPublisher.set(algaeTrigger.getAsBoolean());
        }


        if (unlimitedAmmoTrigger.getAsBoolean() != infiniteAmmo) {
            SimulatedArena.getInstance().resetFieldForAuto();
            infiniteAmmo = !infiniteAmmo;
        }
        SmartDashboard.putBoolean("Class Infinite Ammo", infiniteAmmo);
    }

    /**
     * Runs intake.
     *
     * @return a {@link Command} that runs the intake.
     */
    public Command intake() {
        return RobotBase.isSimulation() ?
                Commands.runEnd(() -> intakeSimulation.startIntake(),
                        () -> intakeSimulation.stopIntake()) :
                Commands.runEnd(() -> motor.set(-ControlConstants.kIntakeSpeed),
                        () -> motor.set(0.0));
    }

    /**
     * Runs intake until the sensor is triggered.
     *
     * @return a {@link Command} that runs intake until the sensor is triggered.
     */
    public Command intakeUntilSensed() {
        return intake()
                .until(coralTrigger);
    }

    /**
     * Runs intake in reverse for algae.
     *
     * @return a {@link Command} that runs intake in reverse for algae.
     */
    public Command intakeAlgae() {
        return
                Commands.runEnd(() -> motor.set(ControlConstants.kIntakeAlgaeSpeed),
                        () -> motor.set(0.0));
    }

    /**
     * Runs intake in reverse for algae until the sensor is triggered.
     *
     * @return a {@link Command} that runs intake in reverse for algae until the sensor is triggered.
     */
    public Command intakeAlgaeUntilSensed() {
        return
                intakeAlgae()
                        .until(algaeTrigger);
    }

    /**
     * Runs shooter.
     *
     * @return a {@link Command} that runs the shooter.
     */
    public Command shoot() {
        DriverStation.reportWarning("Shoot reached.", false);
        return RobotBase.isSimulation() ?
                simShoot() :
                Commands.run(() ->
                        motor.set(ControlConstants.kShootSpeed));
    }

    /**
     * Runs shooter until the sensor is no longer triggered.
     *
     * @return a {@link Command} that r
     */
    public Command shootUntilGone() {
        return shoot()
                .until(coralTrigger.negate());
    }

    /**
     * Runs shooter in reverse for algae.
     *
     * @return a {@link Command} that runs shooter in reverse for algae.
     */
    public Command shootAlgae() {
        return
                Commands.runEnd(
                        () -> motor.set(-ControlConstants.kShootAlgaeSpeed),
                        () -> motor.set(0.0));
    }

    /**
     * Runs shooter in reverse for algae until the sensor is no longer triggered.
     *
     * @return a {@link Command} that runs shooter in reverse for algae until the sensor is no longer triggered.
     */
    public Command shootAlgaeUntilGone() {
        return
                shootAlgae()
                        .until(algaeTrigger);
    }

    /**
     * Adds a game piece to the intake simulation.
     */
    public void addSimCoralToIntake() {
        intakeSimulation.addGamePieceToIntake();
    }

    /**
     * Spawns coral as if it was shot from the real robot. Requires a piece to be in the intake.
     * Has an Unlimited Ammo publisher that makes it so a piece is not required in the intake.
     *
     * @return a {@link Command} that spawns coral as if it was shot from the real robot.
     */
    // TODO Make simShoot more accurately shoot the game piece
    public Command simShoot() {
        DriverStation.reportWarning("simShoot reached.", false);

        SmartDashboard.putBoolean("Infinite Ammo in Method", infiniteAmmo);
        return Commands.runOnce(() -> {
            ReefscapeCoralOnFly coralShot = new ReefscapeCoralOnFly(
                    // Obtain robot position from drive simulation
                    swerve.getMapleDrive().getSimulatedDriveTrainPose().getTranslation(),
                    // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                    new Translation2d(
                            (Arm.HardwareConstants.kArmLength.in(Meters) / 2 +
                                    Elevator.HardwareConstants.kCenterToElevator) - Math.abs(arm.getAngle().in(Rotations) * 1.2),
                            0),
                    // Obtain robot speed from drive simulation
                    swerve.getMapleDrive().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    // Obtain robot facing from drive simulation
                    swerve.getMapleDrive().getSimulatedDriveTrainPose().getRotation(),
                    // The height at which the coral is ejected
                    Inches.of(elevator.getHeight().in(Inches) +
                            Elevator.HardwareConstants.kBottomCarriageToArmInches),
                    // The initial speed of the coral
                    MetersPerSecond.of(4),
                    // The coral is ejected at a 35-degree slope
                    arm.getAngle());
            if (infiniteAmmo) {
            SimulatedArena.getInstance().addGamePieceProjectile(coralShot);
        } else {
            if (coralTrigger.getAsBoolean()) {
                SimulatedArena.getInstance().addGamePieceProjectile(coralShot);
                intakeSimulation.setGamePiecesCount(0);
        }}});
    }

    /**
     * Gets the {@link Trigger} for the coral sensor.
     *
     * @return the {@link Trigger} for the coral sensor.
     */
    public Trigger getCoralTrigger() {
        return coralTrigger;
    }

    /**
     * Gets the {@link Trigger} for the algae sensor.
     *
     * @return the {@link Trigger} for the algae sensor.
     */
    public Trigger getAlgaeTrigger() {
        return algaeTrigger;
    }

    /**
     * The setpoint constants for this arm subsystem.
     */
    public static final class ControlConstants {
        public static final double kIntakeSpeed = .3;
        public static final double kShootSpeed = .65;
        public static final double kIntakeAlgaeSpeed = .5;
        public static final double kShootAlgaeSpeed = 1;
        public static final double kPullBackInSpeed = .02;
    }

    /**
     * The hardware constants for this arm subsystem.
     */
    public static final class HardwareConstants {
        public static final int kMotorID = 11;
        public static final int kCoralSensorID = 4; // DIO
        public static final int kAlgaeSensorID = 0;

        public static final class kProfiledPID {
            public static final double kKp = 4.0;
            public static final double kKi = 0.0;
            public static final double kKd = 0.0;
            public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(360);
            public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(720);
        }

        public static final class kFF {
            public static final double kS = 0.1;
            public static final double kV = 0.1;
            public static final double kG = 0.0;
            public static final double kA = 0.0;
        }
    }
}