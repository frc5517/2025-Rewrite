package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.PoseSelector;

public class SuperStructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private PoseSelector selector;
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private IntakeShooterSubsystem intakeShooter;
    private AddressableLEDSubsystem led;
    public SuperStructure(SwerveSubsystem swerve,
                          PoseSelector selector,
                          ArmSubsystem arm,
                          ElevatorSubsystem elevator,
                          IntakeShooterSubsystem intakeShooter,
                          AddressableLEDSubsystem led) {
        this.swerve = swerve;
        this.selector = selector;
        this.arm = arm;
        this.elevator = elevator;
        this.intakeShooter = intakeShooter;
        this.led = led;
    }

    public Command autoScore(ScoreLevels level) {
        return swerve.driveToReef(selector)
                .alongWith(elevator.setHeight(getElevatorSetpoint(level)))
                .alongWith(arm.setAngle(getArmSetpoint(level))
                        .alongWith(scoreWhenReady(level)));
    }

    public Command autoCollect() {
        return swerve.driveToStation(selector)
                .andThen(intakeShooter.intakeUntilSensed());
    }

    private Command scoreWhenReady(ScoreLevels level) {
        return intakeShooter.shootUntilGone()
                .onlyIf(swerve.atReef(selector)
                        .and(() -> elevator.atHeight(getElevatorSetpoint(level)))
                        .and(() -> arm.atAngle(getArmSetpoint(level))));
    }

    private Angle getArmSetpoint(ScoreLevels levels) {
        return switch (levels) {
            case SCORE_L1 -> Constants.ArmConstants.kL1Setpoint;
            case SCORE_L2 -> Constants.ArmConstants.kL2Setpoint;
            case SCORE_L3 -> Constants.ArmConstants.kL3Setpoint;
            case SCORE_L4 -> Constants.ArmConstants.kL4Setpoint;
            default -> Constants.ArmConstants.kStowSetpoint;
        };
    }

    private Distance getElevatorSetpoint(ScoreLevels levels) {
        return switch (levels) {
            case SCORE_L1 -> Constants.ElevatorConstants.kL1Setpoint;
            case SCORE_L2 -> Constants.ElevatorConstants.kL2Setpoint;
            case SCORE_L3 -> Constants.ElevatorConstants.kL3Setpoint;
            case SCORE_L4 -> Constants.ElevatorConstants.kL4Setpoint;
            default -> Constants.ElevatorConstants.kStowSetpoint;
        };
    }

    public enum ScoreLevels {
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4
    }

}
