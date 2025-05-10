package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.Telemetry;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import yams.gearing.gearbox.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.Telemetry.algaeSensorPublisher;
import static frc.robot.utils.Telemetry.coralSensorPublisher;
import static yams.mechanisms.SmartMechanism.*;

public class IntakeShooterSubsystem extends SubsystemBase {

    private final SparkMax intakeShooterMotor = new SparkMax(IntakeShooterConstants.kMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withMechanismCircumference(Constants.ElevatorConstants.kSprocketCircumference)
            .withGearing(gearing(gearbox(GearBox.Type.VERSA_PLANETARY, 9, 3), sprocket(12, 28)))
            .withClosedLoopController(
                    IntakeShooterConstants.kKp,
                    IntakeShooterConstants.kKi,
                    IntakeShooterConstants.kKd)
            .withFeedforward(new SimpleMotorFeedforward(
                    IntakeShooterConstants.kS, IntakeShooterConstants.kV, IntakeShooterConstants.kA))
            .withTelemetry("Motor", Telemetry.intakeShooterVerbosity)
            .withStatorCurrentLimit(Amps.of(30))
            .withVoltageCompensation(Volts.of(12))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final DigitalInput coralSensor = new DigitalInput(IntakeShooterConstants.kCoralSensorID);
    private final DigitalInput algaeSensor = new DigitalInput(IntakeShooterConstants.kAlgaeSensorID);
    private final Trigger algaeTrigger = new Trigger(() -> !algaeSensor.get());
    // Maple sim stuff
    private final SwerveSubsystem swerve;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private SmartMotorController motor = new SparkWrapper(intakeShooterMotor, DCMotor.getNEO(1), motorConfig);
    private Trigger coralTrigger = new Trigger(coralSensor::get);
    private IntakeSimulation intakeSimulation = null;

    public IntakeShooterSubsystem(SwerveSubsystem drivebase, ElevatorSubsystem elevator, ArmSubsystem arm) {
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
    }

    @Override
    public void periodic() {
        if (Telemetry.robotVerbosity.ordinal() >= Telemetry.RobotTelemetry.LOW.ordinal()) {
            coralSensorPublisher.set(coralTrigger.getAsBoolean());
            algaeSensorPublisher.set(algaeTrigger.getAsBoolean());
        }
    }

    public Command intake() {
        return RobotBase.isSimulation() ?
                Commands.runEnd(() -> intakeSimulation.startIntake(),
                        () -> intakeSimulation.stopIntake()) :
                Commands.runEnd(() -> motor.setDutyCycle(-IntakeShooterConstants.kIntakeSpeed),
                        () -> motor.setDutyCycle(0.0));
    }

    public Command intakeUntilSensed() {
        return intake()
                .until(coralTrigger);
    }

    public Command intakeAlgae() {
        return
                Commands.runEnd(() -> motor.setDutyCycle(IntakeShooterConstants.kIntakeAlgaeSpeed),
                        () -> motor.setDutyCycle(0.0));
    }

    public Command intakeAlgaeUntilSensed() {
        return
                intakeAlgae()
                        .until(algaeTrigger);
    }

    public Trigger getCoralTrigger() {
        return coralTrigger;
    }

    public Trigger getAlgaeTrigger() {
        return algaeTrigger;
    }

    public Command shoot() {
        return RobotBase.isSimulation() ?
                simShoot() :
                Commands.run(() ->
                        motor.setDutyCycle(IntakeShooterConstants.kShootSpeed));
    }

    public Command shootUntilGone() {
        return shoot()
                .until(coralTrigger.negate());
    }

    public Command shootAlgae() {
        return
                Commands.runEnd(
                        () -> motor.setDutyCycle(-IntakeShooterConstants.kShootAlgaeSpeed),
                        () -> motor.setDutyCycle(0.0));
    }

    public Command shootAlgaeUntilGone() {
        return
                shootAlgae()
                        .until(algaeTrigger);
    }

    public void addSimCoralToIntake() {
        intakeSimulation.addGamePieceToIntake();
    }

    public Command simShoot() {
        return runOnce(() -> {
            if (intakeSimulation.getGamePiecesAmount() > 0) {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                // Obtain robot position from drive simulation
                                swerve.getMapleDrive().getSimulatedDriveTrainPose().getTranslation(),
                                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                                new Translation2d(
                                        (Constants.ArmConstants.kArmLength.in(Meters) / 2 +
                                                Constants.ElevatorConstants.kCenterToElevator) - Math.abs(arm.getAngle().in(Rotations) * 1.2),
                                        0),
                                // Obtain robot speed from drive simulation
                                swerve.getMapleDrive().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                // Obtain robot facing from drive simulation
                                swerve.getMapleDrive().getSimulatedDriveTrainPose().getRotation(),
                                // The height at which the coral is ejected
                                Inches.of(elevator.getHeight().in(Inches) +
                                        Constants.ElevatorConstants.kBottomCarriageToArmInches),
                                // The initial speed of the coral
                                MetersPerSecond.of(4),
                                // The coral is ejected at a 35-degree slope
                                arm.getAngle()));
                intakeSimulation.setGamePiecesCount(0);
            }
        });
    }
}