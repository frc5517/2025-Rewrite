package frc.robot.utils.maplesim.opponents.reefscape.kitbot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.maplesim.opponents.SmartOpponent;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.maplesim.MapleConstants.PoseConstants.*;

public class KitBot extends SmartOpponent {
    public KitBot(int id, DriverStation.Alliance alliance) {
        this.opponentMassKG = Optional.of(Kilograms.of(55));
        this.opponentMOI = Optional.of(8.0);
        this.opponentWheelRadius = Optional.of(Inches.of(2));
        this.opponentDriveVelocity = Optional.of(MetersPerSecond.of(8.5));
        this.opponentDriveCOF = Optional.of(1.19);
        this.opponentDriveMotor = Optional.of(DCMotor.getNEO(1)
                .withReduction(8.14));
        this.opponentDriveCurrentLimit = Optional.of(40.0);
        this.opponentNumDriveMotors = Optional.of(1);
        this.opponentTrackWidth = Optional.of(Inches.of(23));
        this.robotName = "KitBot";
        setupOpponent(id, alliance);
    }

    @Override
    public void buildBehaviorChooser(int id, DriverStation.Alliance alliance) {
        super.buildBehaviorChooser(id, alliance);
        behaviorChooser.ifPresent(
                chooser -> chooser.addOption("Joystick Drive", Commands.runOnce(() -> setState(States.JOYSTICK))));
    }

    /**
     * @return
     */
    @Override
    public Pose2d getScorePose() {
        this.scoreTarget = Optional.of(((int) Math.round(Math.random() * 11)));
        Pose2d targetPose = switch (scoreTarget.get()) {
            case 0 -> REEF_SOUTH_LEFT_POSE;
            case 1 -> REEF_SOUTH_RIGHT_POSE;
            case 2 -> REEF_SOUTHEAST_LEFT_POSE;
            case 3 -> REEF_SOUTHEAST_RIGHT_POSE;
            case 4 -> REEF_NORTHEAST_LEFT_POSE;
            case 5 -> REEF_NORTHEAST_RIGHT_POSE;
            case 6 -> REEF_NORTH_LEFT_POSE;
            case 7 -> REEF_NORTH_RIGHT_POSE;
            case 8 -> REEF_NORTHWEST_LEFT_POSE;
            case 9 -> REEF_NORTHWEST_RIGHT_POSE;
            case 10 -> REEF_SOUTHWEST_LEFT_POSE;
            case 11 -> REEF_SOUTHWEST_RIGHT_POSE;
            default -> Pose2d.kZero;
        };
        return ifShouldFlip(targetPose);
    }
}
