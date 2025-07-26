package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public class MapleConstants {
    public static class PoseConstants {
        /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
        public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[]
                {
                        new Pose2d(-6, 0, new Rotation2d()),
                        new Pose2d(-5, 0, new Rotation2d()),
                        new Pose2d(-4, 0, new Rotation2d()),
                        new Pose2d(-3, 0, new Rotation2d()),
                        new Pose2d(-2, 0, new Rotation2d())
                };
        public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[]
                {
                        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
                        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
                        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.6, 6, new Rotation2d()),
                        new Pose2d(1.6, 4, new Rotation2d())
                };

        public static final Transform2d STATION_OFFSET = new Transform2d(
                Units.inchesToMeters(-17),
                Units.inchesToMeters(0),
                Rotation2d.kZero);
        public static final Transform2d SLOT_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(24), // Added several times to achieve all 5 poses.
                Rotation2d.kZero);
        public static final Transform2d SLOT_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(-30), // Added several times to achieve all 5 poses.
                Rotation2d.kZero);
        public static final Pose2d BARGE_NET_POSE =
                new Pose2d(
                        7.5,
                        2,
                        Rotation2d.fromDegrees(0));
        public static final Pose2d LEFT_STATION_CENTER_POSE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(125.989));
        public static final Pose2d RIGHT_STATION_CENTER_POSE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(234.011));
        public static final Pose2d LEFT_STATION_POSE = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d RIGHT_STATION_POSE = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d SOUTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(0));
        public static final Pose2d SOUTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(300));
        public static final Pose2d NORTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(240));
        public static final Pose2d NORTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(180));
        public static final Pose2d NORTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(120));
        public static final Pose2d SOUTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(60));
        public static final Transform2d BRANCH_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from the reef.
                Units.inchesToMeters(13 / 2.0), // Offset to left branch.
                Rotation2d.kZero);
        public static final Transform2d BRANCH_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from the reef.
                Units.inchesToMeters(-13 / 2.0), // Offset to the right branch.
                Rotation2d.kZero);
        public static final Pose2d REEF_NORTH_LEFT_POSE =
                NORTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTH_RIGHT_POSE =
                NORTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHEAST_LEFT_POSE =
                NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHEAST_RIGHT_POSE =
                NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHWEST_LEFT_POSE =
                NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHWEST_RIGHT_POSE =
                NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTH_LEFT_POSE =
                SOUTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTH_RIGHT_POSE =
                SOUTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHEAST_LEFT_POSE =
                SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHEAST_RIGHT_POSE =
                SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHWEST_LEFT_POSE =
                SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHWEST_RIGHT_POSE =
                SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);

        public static final List<Pose2d> targetPoses = new ArrayList<>();
        public static final List<Pose2d> stationPoses = new ArrayList<>();

        static {
            targetPoses.add(0, REEF_SOUTH_LEFT_POSE);
            targetPoses.add(1, REEF_SOUTH_RIGHT_POSE);
            targetPoses.add(2, REEF_SOUTHEAST_LEFT_POSE);
            targetPoses.add(3, REEF_SOUTHEAST_RIGHT_POSE);
            targetPoses.add(4, REEF_NORTHEAST_LEFT_POSE);
            targetPoses.add(5, REEF_NORTHEAST_RIGHT_POSE);
            targetPoses.add(6, REEF_NORTH_LEFT_POSE);
            targetPoses.add(7, REEF_NORTH_RIGHT_POSE);
            targetPoses.add(8, REEF_NORTHWEST_LEFT_POSE);
            targetPoses.add(9, REEF_NORTHWEST_RIGHT_POSE);
            targetPoses.add(10, REEF_SOUTHWEST_LEFT_POSE);
            targetPoses.add(11, REEF_SOUTHWEST_RIGHT_POSE);
            targetPoses.add(12, BARGE_NET_POSE);

            stationPoses.add(0, LEFT_STATION_POSE);
            stationPoses.add(1, RIGHT_STATION_POSE);
        }
    }
}