package frc.robot.utils.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.utils.robot.Telemetry.*;

public class PoseSelector extends SubsystemBase {

    private final ReefPose[] poses = ReefPose.values();
    private final ReefSide[] sides = ReefSide.values();
    private final LeftOrRight[] leftOrRights = LeftOrRight.values();
    private final StationSlot[] stationSlots = StationSlot.values();
    private final MutAngle selectedPose = Degrees.mutable(0);
    private final SwerveSubsystem swerve;
    private ReefPose reefPose = ReefPose.NORTH_LEFT;
    private ReefSide reefSide = ReefSide.NORTH;
    private LeftOrRight leftOrRight = LeftOrRight._LEFT;
    private StationSlot stationSlot = StationSlot.POSE_1;
    private StationPose stationPose = StationPose.POSE_1_LEFT;

    public PoseSelector(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        selectedReefPosePublisher.set(reefPose.toString());
        selectedReefSidePublisher.set(reefSide.toString());
        selectedReefBranchPublisher.set(leftOrRight.toString());
        selectedReefPoseCompassPublisher.set(selectedPose.in(Degrees));

        selectedStationPosePublisher.set(stationPose.toString());
        selectedStationSlotPublisher.set(stationSlot.toString());
        selectedStationSidePublisher.set(leftOrRight.toString());
    }

    /**
     * Cycles poses right.
     */
    public void cycleReefPoseRight() {
        int ordinalPoseUp = (reefPose.ordinal() + 1) % poses.length;
        reefPose = poses[ordinalPoseUp];
        updatePoseData(false);
    }

    /**
     * Cycles poses right.
     */
    public void cycleReefPoseLeft() {
        int ordinalPoseDown = (reefPose.ordinal() - 1) % poses.length;
        if (ordinalPoseDown == -1) {
            ordinalPoseDown = poses.length - 1;
        }
        reefPose = poses[ordinalPoseDown];
        updatePoseData(false);
    }

    /**
     * Cycles where to go to on the coral station.
     */
    public void cycleStationSlotUp() {
        int ordinalSlotUp = (stationSlot.ordinal() + 1) % stationSlots.length;
        stationSlot = stationSlots[ordinalSlotUp];
        updateStationPose();
    }

    /**
     * Cycles where to go to on the coral station.
     */
    public void cycleStationSlotDown() {
        int ordinalSlotDown = (stationSlot.ordinal() - 1) % stationSlots.length;
        if (ordinalSlotDown == -1) {
            ordinalSlotDown = stationSlots.length - 1;
        }
        stationSlot = stationSlots[ordinalSlotDown];
        updateStationPose();
    }

    /**
     * Updates reef selection with reefSide.
     *
     * @param reefSide {@link ReefSide} to use.
     */
    public void selectReefSide(ReefSide reefSide) {
        this.reefSide = reefSide;
        updatePoseData(true);
    }

    /**
     * Updates reef selection with left or right side.
     *
     * @param isRight whether to select right or left.
     */
    public void selectLR(boolean isRight) {
        if (isRight) {
            leftOrRight = LeftOrRight._RIGHT;
        } else {
            leftOrRight = LeftOrRight._LEFT;
        }
        updatePoseData(true);
    }

    /**
     * Updates the selectedPose angle and current selected pose if fed separate side and branch data.
     *
     * @param together whether to update from branch and side data or directly from current selected pose.
     */
    public void updatePoseData(boolean together) {
        if (together) {
            reefPose = ReefPose.valueOf(reefSide.toString() + leftOrRight.toString());
        } else {
            // Determines which side pose is on.
            reefSide = sides[reefPose.ordinal() / 2];
            // Determines if selected pose is left or right.
            leftOrRight = leftOrRights[reefPose.ordinal() % 2 == 0 ? 0 : 1];
        }

        selectedPose.mut_replace(((reefSide.ordinal() * 360) / 6.0) +
                (leftOrRight.ordinal() == 0 ? -10 : 10), Degrees);
    }

    public void updateStationPose() {
        stationPose = StationPose.valueOf(stationSlot.toString() + leftOrRight.toString());
    }

    /**
     * @return selected reef {@link Pose2d}.
     */
    public Pose2d selectedReefPose() {
        return switch (reefPose) {
            case NORTH_LEFT -> Constants.DrivebaseConstants.REEF_NORTH_LEFT_POSE;
            case NORTH_RIGHT -> Constants.DrivebaseConstants.REEF_NORTH_RIGHT_POSE;
            case NORTHEAST_LEFT -> Constants.DrivebaseConstants.REEF_NORTHEAST_LEFT_POSE;
            case NORTHEAST_RIGHT -> Constants.DrivebaseConstants.REEF_NORTHEAST_RIGHT_POSE;
            case NORTHWEST_LEFT -> Constants.DrivebaseConstants.REEF_NORTHWEST_LEFT_POSE;
            case NORTHWEST_RIGHT -> Constants.DrivebaseConstants.REEF_NORTHWEST_RIGHT_POSE;
            case SOUTH_LEFT -> Constants.DrivebaseConstants.REEF_SOUTH_LEFT_POSE;
            case SOUTH_RIGHT -> Constants.DrivebaseConstants.REEF_SOUTH_RIGHT_POSE;
            case SOUTHEAST_LEFT -> Constants.DrivebaseConstants.REEF_SOUTHEAST_LEFT_POSE;
            case SOUTHEAST_RIGHT -> Constants.DrivebaseConstants.REEF_SOUTHEAST_RIGHT_POSE;
            case SOUTHWEST_LEFT -> Constants.DrivebaseConstants.REEF_SOUTHWEST_LEFT_POSE;
            case SOUTHWEST_RIGHT -> Constants.DrivebaseConstants.REEF_SOUTHWEST_RIGHT_POSE;
            default -> swerve.getPose();
        };
    }

    public Pose2d flippedReefPose() {
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(selectedReefPose()) : selectedReefPose();
    }

    public Pose2d selectedStationPose() {
        return switch (stationPose) {
            case POSE_1_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_1;
            case POSE_2_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_2;
            case POSE_3_LEFT -> Constants.DrivebaseConstants.LEFT_STATION_POSE_3;
            case POSE_1_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_1;
            case POSE_2_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_2;
            case POSE_3_RIGHT -> Constants.DrivebaseConstants.RIGHT_STATION_POSE_3;
            default -> swerve.getPose();
        };
    }

    public Pose2d flippedStationPose() {
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(selectedStationPose()) : selectedStationPose();
    }

    public Pose2d selectedCagePose() {
        return switch (stationSlot) {
            case POSE_1 -> Constants.DrivebaseConstants.LEFT_CAGE_POSE;
            case POSE_2 -> Constants.DrivebaseConstants.MIDDLE_CAGE_POSE;
            case POSE_3 -> Constants.DrivebaseConstants.RIGHT_CAGE_POSE;
            default -> swerve.getPose();
        };
    }

    public Pose2d flippedCagePose() {
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(selectedCagePose()) : selectedCagePose();
    }

    public Pose2d selectedAlgaePose() {
        return switch (reefSide) {
            case NORTH -> Constants.DrivebaseConstants.ALGAE_NORTH;
            case NORTHEAST -> Constants.DrivebaseConstants.ALGAE_NORTHEAST;
            case SOUTHEAST -> Constants.DrivebaseConstants.ALGAE_SOUTHEAST;
            case SOUTH -> Constants.DrivebaseConstants.ALGAE_SOUTH;
            case SOUTHWEST -> Constants.DrivebaseConstants.ALGAE_SOUTHWEST;
            case NORTHWEST -> Constants.DrivebaseConstants.ALGAE_NORTHWEST;
            default -> swerve.getPose();
        };
    }

    public Pose2d flippedAlgaePose() {
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(selectedAlgaePose()) : selectedAlgaePose();
    }

    public Pose2d flippedProcessorPose() {
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(Constants.DrivebaseConstants.PROCESSOR) : Constants.DrivebaseConstants.PROCESSOR;
    }

    public Pose2d flippedStartRight() {
        Pose2d startRight = new Pose2d(7.25, 2, Rotation2d.k180deg);
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(startRight) : startRight;
    }

    public enum ReefSide {
        NORTH,
        NORTHEAST,
        SOUTHEAST,
        SOUTH,
        SOUTHWEST,
        NORTHWEST,
    }

    public enum LeftOrRight {
        _RIGHT,
        _LEFT,
    }

    public enum ReefPose {
        NORTH_RIGHT,
        NORTH_LEFT,
        NORTHEAST_RIGHT,
        NORTHEAST_LEFT,
        SOUTHEAST_RIGHT,
        SOUTHEAST_LEFT,
        SOUTH_RIGHT,
        SOUTH_LEFT,
        SOUTHWEST_RIGHT,
        SOUTHWEST_LEFT,
        NORTHWEST_RIGHT,
        NORTHWEST_LEFT,
    }

    public enum StationSlot {
        POSE_1,
        POSE_2,
        POSE_3,
    }

    public enum StationPose {
        POSE_1_LEFT,
        POSE_2_LEFT,
        POSE_3_LEFT,

        POSE_1_RIGHT,
        POSE_2_RIGHT,
        POSE_3_RIGHT,
    }

}
