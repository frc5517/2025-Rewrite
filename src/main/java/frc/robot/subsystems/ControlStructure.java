package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.robot.PoseSelector;

import frc.robot.subsystems.AddressableLEDSubsystem.*;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class ControlStructure extends SubsystemBase {
    private final SwerveSubsystem swerve;
    private final PoseSelector selector;
    private final Arm arm;
    private final Elevator elevator;
    private final IntakeShooter intakeShooter;
    private final AddressableLEDSubsystem led;
    private MechanismLigament2d elevatorMech;
    private MechanismLigament2d armMech;

    /**
     * Initializer for the Control superstructure.
     * This is the "brain" that controls the body.
     * Calls for all subsystems.
     */
    public ControlStructure(SwerveSubsystem swerve,
                            PoseSelector selector,
                            Arm arm,
                            Elevator elevator,
                            IntakeShooter intakeShooter,
                            AddressableLEDSubsystem led) {
        this.swerve = swerve;
        this.selector = selector;
        this.arm = arm;
        this.elevator = elevator;
        this.intakeShooter = intakeShooter;
        this.led = led;
        if (RobotBase.isSimulation()) {
            setupMech();
        }
    }

    @Override
    public void simulationPeriodic() {
        updateMech();
    }

    /**
     * Initializes the {@link Mechanism2d} simulation.
     */
    public void setupMech() {
        Mechanism2d mechWindow = new Mechanism2d(
                Elevator.HardwareConstants.kTopHardLimit.in(Meters) * 2,
                Elevator.HardwareConstants.kTopHardLimit.in(Meters) * 2);
        MechanismRoot2d mechRoot = mechWindow.getRoot("Mech Root",
                Elevator.SimConstants.kMechanismPosition.kXFrontPositive.in(Meters),
                Elevator.SimConstants.kMechanismPosition.kYLeftPositive.in(Meters));
        elevatorMech = mechRoot.append(new MechanismLigament2d(
                "Elevator",
                Elevator.SimConstants.kSimStartingHeight.in(Meters),
                90, 6, new Color8Bit(Color.kDarkRed)));
        armMech = elevatorMech.append(new MechanismLigament2d(
                "Arm",
                Arm.HardwareConstants.kArmLength.in(Meters),
                0, 6, new Color8Bit(Color.kPaleVioletRed)));
        SmartDashboard.putData("RobotTelemetry/Mech2D", mechWindow);
    }

    /**
     * Updates the {@link Mechanism2d} simulation.
     */
    public void updateMech() {
        elevatorMech.setLength(elevator.getHeight().in(Meters));
        armMech.setAngle(arm.getAngle().in(Degrees) - 90);
    }

    /**
     * Runs a command sequence to automatically drive to selected pose and score on the reef.
     * @param level which level to score the coral.
     * @param speedBoost whether to increase speed or not.
     * @return a {@link Command} sequence to automatically drive to selected pose and score on the reef.
     */
    public Command autoScore(ScoreLevels level, Trigger speedBoost) {
        return swerve.driveToReef(selector, speedBoost, 1)
                .alongWith(score(level));
    }

    /**
     * Runs a command sequence to automatically collect from selected coral station slot.
     * @param speedBoost whether to increase speed or not.
     * @return a {@link Command} sequence to automatically collect from selected coral station slot.
     */
    public Command autoCollect(Trigger speedBoost) {
        return swerve.driveToStation(selector, speedBoost, 1)
                .alongWith(collect());
    }

    /**
     * Runs a command that moves structures to coral station poses and finishes when a coral is detected.
     * @return a {@link Command} that moves structures to coral station poses and finishes when a coral is detected.
     */
    public Command collect() {
        return elevator.setHeight(Elevator.ControlConstants.kStationSetpoint)
                .alongWith(arm.setAngle(Arm.ControlConstants.kStationSetpoint))
                .alongWith(intakeShooter.intakeUntilSensed());
    }

    /**
     * Runs a command that moves structures to {@link ScoreLevels} and shoots when all subsystems are ready.
     * @param level which level to score the coral.
     * @return a {@link Command} that moves structures to {@link ScoreLevels} and shoots when all subsystems are ready.
     */
    private Command score(ScoreLevels level) {
        return elevator.setHeight(getElevatorSetpoint(level))
                .alongWith(arm.setAngle(getArmSetpoint(level)))
                .alongWith(intakeShooter.shoot().onlyIf(readyToScore(level)));
    }

    /**
     * Runs a command that moves structures to {@link ScoreLevels} and shoots when all systems are ready.
     * @param level which level to score the coral.
     * @param readyToScore whether the systems are ready to score.
     * @return a {@link Command} that moves structures to {@link ScoreLevels} and shoots when all subsystems are ready.
     */
    public Command manualScore(ScoreLevels level, Trigger readyToScore) {
        return elevator.setHeight(getElevatorSetpoint(level))
                .alongWith(arm.setAngle(getArmSetpoint(level)))
                .alongWith(intakeShooter.shoot()
                        .onlyWhile(readyToScore));
    }

    /**
     * Runs a command that only shoots if arm and elevator are at their setpoints.
     * @param level the reef level being scored.
     * @return a {@link Command} that only shoots if arm and elevator are at their setpoints.
     */
    private BooleanSupplier readyToScore(ScoreLevels level) {
        // TODO Testing telemetry Data, readyToScore isn't updating
        SmartDashboard.putBoolean("Arm at Angle", arm.atAngle(getArmSetpoint(level)));
        SmartDashboard.putBoolean("Elevator at Height", elevator.atHeight(getElevatorSetpoint(level)));
        SmartDashboard.putBoolean("Swerve not moving", swerve.isMoving(0.01));
        SmartDashboard.putBoolean("Swerve done running", !swerve.toPoseIsRunning());
        return () -> arm.atAngle(getArmSetpoint(level))
                && elevator.atHeight(getElevatorSetpoint(level))
                && (!swerve.toPoseIsRunning());
    }

    /**
     * Runs a command that sets all LEDs to scrolling rainbow.
     * @return a {@link Command} that sets all LEDs to scrolling rainbow.
     */
    private Command setLEDRainbow() {
        return Commands.run(() -> {
            led.runLED(LEDViews.BOTH, LEDModes.RAINBOW);
        }).finallyDo(() -> led.runLED(LEDViews.BOTH, LEDModes.OFF));
    }

    /**
     * The arm setpoint for a given reef level.
     * @param levels which setpoint level.
     * @return the arm setpoint for a given reef level.
     */
    private Angle getArmSetpoint(ScoreLevels levels) {
        return switch (levels) {
            case SCORE_L1 -> Arm.ControlConstants.kL1Setpoint;
            case SCORE_L2 -> Arm.ControlConstants.kL2Setpoint;
            case SCORE_L3 -> Arm.ControlConstants.kL3Setpoint;
            case SCORE_L4 -> Arm.ControlConstants.kL4Setpoint;
        };
    }

    /**
     * The elevator setpoint for a given reef level.
     * @param levels which setpoint level.
     * @return the elevator setpoint for a given reef level.
     */
    private Distance getElevatorSetpoint(ScoreLevels levels) {
        return switch (levels) {
            case SCORE_L1 -> Elevator.ControlConstants.kL1Setpoint;
            case SCORE_L2 -> Elevator.ControlConstants.kL2Setpoint;
            case SCORE_L3 -> Elevator.ControlConstants.kL3Setpoint;
            case SCORE_L4 -> Elevator.ControlConstants.kL4Setpoint;
        };
    }

    private boolean isDoneScoring(ScoreLevels level) {
        return arm.atAngle(getArmSetpoint(level))
                && elevator.atHeight(getElevatorSetpoint(level))
                && intakeShooter.getCoralTrigger().negate().getAsBoolean();
    }

    /**
     * Which level of the reef to score.
     */
    public enum ScoreLevels {
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4
    }
}
