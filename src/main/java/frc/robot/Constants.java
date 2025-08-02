// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final double DEADBAND = 0.1;
    }

    public static final class DrivebaseConstants {
        public static final double MAX_SPEED = Units.feetToMeters(11.5);
        public static final double kScaleSpeedMax = 0.2;
        public static final double kScaleSpeedMin = 0.1;
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        // Drive to pose speeds
        public static final double kDriveToReef = .65;
        public static final double kDriveToStation = .65;
        public static final double kDriveToProcessor = .65;
        public static final double kDriveToAlgae = .65;
        public static final double kDriveToCage = .65;

        // Drive to pose constants
        // Offset used to update the pose during driveToPose
        public static final Transform2d kToPoseUpdateOffset = new Transform2d(
                Units.inchesToMeters(-24),
                Units.inchesToMeters(0),
                Rotation2d.kZero);
        // Tolerance distance until going to PPHolonomic PID
        public static final double kDistanceUntilPID = Units.inchesToMeters(1);
        public static final double kRotationGoalBeforePID = 1;
        public static final LinearVelocity kPathfindEndGoalVelocity = MetersPerSecond.of(0);
        public static final Distance kTranslationTolerance = Inches.of(1);
        public static final Angle kRotationTolerance = Degrees.of(1); // Degrees

        // Pathplanner holonomic controller
        public static final PIDConstants kPPTranslationPID = new PIDConstants(5.2, 0.0, 0.0);
        public static final PIDConstants kPPRotationPID = new PIDConstants(5.0, 0.0, 0.0);

        //
        // Pose offsets below. Proceed with caution!
        //
        public static final Transform2d PROCESSOR_OFFSET = new Transform2d(
                Units.inchesToMeters(-30),
                Units.inchesToMeters(0),
                Rotation2d.kZero);
        public static final Transform2d ALGAE_OFFSET = new Transform2d(
                Units.inchesToMeters(-30),
                Units.inchesToMeters(0),
                Rotation2d.kZero);
        public static final Transform2d BRANCH_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from reef.
                Units.inchesToMeters(13 / 2.0), // Offset to left branch.
                Rotation2d.kZero);
        public static final Transform2d BRANCH_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from reef.
                Units.inchesToMeters(-13 / 2.0), // Offset to right branch.
                Rotation2d.kZero);
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
        public static final Transform2d CAGE_OFFSET = new Transform2d(
                Units.inchesToMeters(-32 - 20), // cage pose - offset
                Units.inchesToMeters(0),
                Rotation2d.k180deg);

        //
        // Constant poses below, proceed with EXTREME CAUTION!!
        //
        public static final Pose2d kProcessorCenter =
                new Pose2d(
                        Units.inchesToMeters(235.726),
                        0,
                        Rotation2d.fromDegrees(-90));
        public static final Pose2d PROCESSOR = kProcessorCenter
                .plus(PROCESSOR_OFFSET);
        public static final Pose2d kLeftCage =
                new Pose2d(
                        Units.inchesToMeters(345.428),
                        Units.inchesToMeters(286.779),
                        Rotation2d.kZero);
        public static final Pose2d kMiddleCage =
                new Pose2d(
                        Units.inchesToMeters(345.428),
                        Units.inchesToMeters(242.855),
                        Rotation2d.kZero);
        public static final Pose2d kRightCage =
                new Pose2d(
                        Units.inchesToMeters(345.428),
                        Units.inchesToMeters(199.947),
                        Rotation2d.kZero);
        public static final Pose2d LEFT_CAGE_POSE = kLeftCage
                .plus(CAGE_OFFSET);
        public static final Pose2d MIDDLE_CAGE_POSE = kMiddleCage
                .plus(CAGE_OFFSET);
        public static final Pose2d RIGHT_CAGE_POSE = kRightCage
                .plus(CAGE_OFFSET);
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
        public static final Pose2d LEFT_STATION_POSE_1 = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_LEFT);
        public static final Pose2d LEFT_STATION_POSE_2 = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d LEFT_STATION_POSE_3 = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_RIGHT);
        public static final Pose2d RIGHT_STATION_POSE_1 = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_RIGHT);
        public static final Pose2d RIGHT_STATION_POSE_2 = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d RIGHT_STATION_POSE_3 = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET)
                .plus(SLOT_OFFSET_LEFT);
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

        public static final Pose2d ALGAE_NORTH =
                NORTH_FACE_POSE.plus(ALGAE_OFFSET);
        public static final Pose2d ALGAE_NORTHEAST =
                NORTHEAST_FACE_POSE.plus(ALGAE_OFFSET);
        public static final Pose2d ALGAE_NORTHWEST =
                NORTHWEST_FACE_POSE.plus(ALGAE_OFFSET);
        public static final Pose2d ALGAE_SOUTH =
                SOUTH_FACE_POSE.plus(ALGAE_OFFSET);
        public static final Pose2d ALGAE_SOUTHEAST =
                SOUTHEAST_FACE_POSE.plus(ALGAE_OFFSET);
        public static final Pose2d ALGAE_SOUTHWEST =
                SOUTHWEST_FACE_POSE.plus(ALGAE_OFFSET);
    }

    public static final class VisionConstants {
        // Coordinate system, makes x, y, and z easy.
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        // Reef Camera Constants
        public static final Rotation3d kReefCamRotation = new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(0),
                Units.degreesToRadians(0));
        public static final Translation3d kReefCamPosition = new Translation3d(
                Units.inchesToMeters(3),
                Units.inchesToMeters(0),
                Units.inchesToMeters(13));

        // Climb Camera Constants
        public static final Rotation3d kClimbCamRotation = new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(35),
                Units.degreesToRadians(0));
        public static final Translation3d kClimbCamPosition = new Translation3d(
                Units.inchesToMeters(4),
                Units.inchesToMeters(0),
                Units.inchesToMeters(35));

        // Intake Camera Constants
        public static final Rotation3d kIntakeCamRotation = new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-35),
                Units.degreesToRadians(0));
        public static final Translation3d kIntakeCamPosition = new Translation3d(
                Units.inchesToMeters(4),
                Units.inchesToMeters(-6.75),
                Units.inchesToMeters(34));
    }

}
