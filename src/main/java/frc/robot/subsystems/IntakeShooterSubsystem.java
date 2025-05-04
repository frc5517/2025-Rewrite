package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import yams.gearing.gearbox.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static yams.mechanisms.SmartMechanism.*;

public class IntakeShooterSubsystem extends SubsystemBase {

    private SparkMax intakeShooterMotor = new SparkMax(IntakeShooterConstants.kMotorID, SparkLowLevel.MotorType.kBrushless);
    private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withMechanismCircumference(Constants.ElevatorConstants.kSprocketCircumference)
            .withGearing(gearing(gearbox(GearBox.Type.VERSA_PLANETARY, 9, 3), sprocket( 12, 28)))
            .withClosedLoopController(
                    IntakeShooterConstants.kKp,
                    IntakeShooterConstants.kKi,
                    IntakeShooterConstants.kKd)
            .withFeedforward(new SimpleMotorFeedforward(
                    IntakeShooterConstants.kS, IntakeShooterConstants.kV, IntakeShooterConstants.kA))
            .withTelemetry("IntakeShooterMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(30))
            .withVoltageCompensation(Volts.of(12))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private SmartMotorController motor = new SparkWrapper(intakeShooterMotor, DCMotor.getNEO(1), motorConfig);

    private final DigitalInput coralSensor = new DigitalInput(IntakeShooterConstants.kCoralSensorID);
    private final DigitalInput algaeSensor = new DigitalInput(IntakeShooterConstants.kAlgaeSensorID);

    private Trigger coralTrigger = new Trigger(coralSensor::get);
    private final Trigger algaeTrigger = new Trigger(() -> !algaeSensor.get());

    // Maple sim stuff
    private final SwerveSubsystem swerve;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private IntakeSimulation intakeSimulation = null;

    StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructArrayTopic("Coral Array",
                    Pose3d.struct)
            .publish();
    StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructArrayTopic("Algae Array",
                    Pose3d.struct)
            .publish();

    public IntakeShooterSubsystem(SwerveSubsystem drivebase, ElevatorSubsystem elevator, ArmSubsystem arm)
    {
        this.swerve = drivebase;
        this.elevator = elevator;
        this.arm = arm;

        if (RobotBase.isSimulation()) {
            this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                    "Coral",
                    drivebase.getMapleDrive(),
                    Inches.of(5),
                    IntakeSimulation.IntakeSide.FRONT,
                    1
            );
            SimulatedArena.getInstance().resetFieldForAuto();
            coralTrigger = new Trigger(() -> intakeSimulation.getGamePiecesAmount() > 0);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral Trigger", coralTrigger.getAsBoolean());
    }

    public Command intake() {
        return RobotBase.isSimulation() ?
                Commands.runOnce(this::addSimCoralToIntake) :
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

    @Override
    public void simulationPeriodic() {
        // Get the positions of the notes (both on the field and in the air);
        coralPoses.set(SimulatedArena.getInstance()
                .getGamePiecesByType("Coral")
                .toArray(Pose3d[]::new)
        );
        algaePoses.set(SimulatedArena.getInstance()
                .getGamePiecesByType("Algae")
                .toArray(Pose3d[]::new)
        );
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
            }});
    }
}