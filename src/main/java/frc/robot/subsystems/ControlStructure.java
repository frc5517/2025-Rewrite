package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.PoseSelector;

public class ControlStructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private PoseSelector selector;
    private Arm arm;
    private Elevator elevator;
    private IntakeShooter intakeShooter;
    private AddressableLEDSubsystem led;

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
    }

    public Command autoScore(ScoreLevels level, Trigger speedBoost) {
        return swerve.driveToReef(selector, speedBoost, 1)
                .alongWith(elevator.setHeight(getElevatorSetpoint(level)))
                .alongWith(arm.setAngle(getArmSetpoint(level)))
                .alongWith(setLEDRainbow())
                .alongWith(scoreWhenReady(level));
    }

    public Command autoCollect(Trigger speedBoost) {
        return swerve.driveToStation(selector, speedBoost, 1)
                .alongWith(setLEDRainbow())
                .andThen(intakeShooter.intakeUntilSensed());
    }

    private Command scoreWhenReady(ScoreLevels level) {
        return intakeShooter.shootUntilGone()
                .onlyIf(swerve.atReef(selector)
                        .and(() -> elevator.atHeight(getElevatorSetpoint(level)))
                        .and(() -> arm.atAngle(getArmSetpoint(level))));
    }

    private Command setLEDRainbow() {
        return Commands.run(() -> {
            led.runPatternBoth(AddressableLEDSubsystem.ControlConstants.rainbow, AddressableLEDSubsystem.ControlConstants.rainbow);
        }).finallyDo(() -> led.runPatternBoth(LEDPattern.kOff, LEDPattern.kOff));
    }

    private Angle getArmSetpoint(ScoreLevels levels) {
        return switch (levels) {
            case SCORE_L1 -> Arm.ControlConstants.kL1Setpoint;
            case SCORE_L2 -> Arm.ControlConstants.kL2Setpoint;
            case SCORE_L3 -> Arm.ControlConstants.kL3Setpoint;
            case SCORE_L4 -> Arm.ControlConstants.kL4Setpoint;
        };
    }

    private Distance getElevatorSetpoint(ScoreLevels levels) {
        return switch (levels) {
            case SCORE_L1 -> Elevator.ControlConstants.kL1Setpoint;
            case SCORE_L2 -> Elevator.ControlConstants.kL2Setpoint;
            case SCORE_L3 -> Elevator.ControlConstants.kL3Setpoint;
            case SCORE_L4 -> Elevator.ControlConstants.kL4Setpoint;
        };
    }

    public enum ScoreLevels {
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4
    }

}
