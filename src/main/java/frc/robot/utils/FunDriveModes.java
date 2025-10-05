package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj.drive.DifferentialDrive.*;

public class FunDriveModes {
    public static Command differentialDrive(DifferentialType type, SwerveSubsystem swerve, CommandXboxController driverXbox, double deadband) {
        // Module number for kinematics, usually 0 to 3. Front left -> front right -> back left -> back right
        SwerveModuleState[] states = new SwerveModuleState[4];
        double maxSpeed = swerve.getSwerveDrive().getMaximumChassisVelocity();
        return Commands.runEnd(() -> {
            WheelSpeeds speeds = switch (type) {
                case TANK ->
                        tankDriveIK(-MathUtil.applyDeadband(driverXbox.getLeftY(), deadband), -MathUtil.applyDeadband(driverXbox.getRightY(), deadband), false);
                case ARCADE ->
                        arcadeDriveIK(-MathUtil.applyDeadband(driverXbox.getLeftY(), deadband), -MathUtil.applyDeadband(driverXbox.getRightX(), deadband), false);
                case CURVATURE ->
                        curvatureDriveIK(-MathUtil.applyDeadband(driverXbox.getLeftY(), deadband), -MathUtil.applyDeadband(driverXbox.getRightX(), deadband), false);
            };
            double leftSpeeds = speeds.left * maxSpeed;
            double rightSpeeds = speeds.right * maxSpeed;

            states[0] = new SwerveModuleState(leftSpeeds, Rotation2d.kZero);
            states[2] = new SwerveModuleState(leftSpeeds, Rotation2d.kZero);
            states[1] = new SwerveModuleState(rightSpeeds, Rotation2d.kZero);
            states[3] = new SwerveModuleState(rightSpeeds, Rotation2d.kZero);

            swerve.setRawModuleStates(states, true);
        }, () -> {
            swerve.drive(new ChassisSpeeds());
        }, swerve);
    }

    public static Command carDrive(CarType type, SwerveSubsystem swerve, CommandXboxController driverXbox, double deadband) {
        // Module number for kinematics, usually 0 to 3. Front left -> front right -> back left -> back right
        SwerveModuleState[] states = new SwerveModuleState[4];
        double maxSpeed = swerve.getSwerveDrive().getMaximumChassisVelocity();
        Angle turningLimit = Degrees.of(50);
        swerve.setMotorBrake(false);
        return Commands.runEnd(() -> {
            double throttle = -driverXbox.getLeftY() * maxSpeed;
            Rotation2d turn = Rotation2d.fromDegrees(turningLimit.times(-driverXbox.getRightX()).in(Degrees));

            switch (type) {
                case FWD -> {
                    states[0] = new SwerveModuleState(throttle, turn);
                    states[1] = new SwerveModuleState(throttle, turn);
                    states[2] = new SwerveModuleState(0, Rotation2d.kZero);
                    states[3] = new SwerveModuleState(0, Rotation2d.kZero);
                }
                case RWD -> {
                    double speedToTurn = Math.abs(turn.getDegrees()) > 2 ? Math.abs(turn.times(throttle).div(2).getRotations()) : 0; // Seems to need speed to move wheel to angle, may be a sim bug.
                    states[0] = new SwerveModuleState(speedToTurn, turn);
                    states[1] = new SwerveModuleState(speedToTurn, turn);
                    states[2] = new SwerveModuleState(throttle, Rotation2d.kZero);
                    states[3] = new SwerveModuleState(throttle, Rotation2d.kZero);
                }
            }

            swerve.setRawModuleStates(states, true);
        }, () -> {
            swerve.setMotorBrake(true);
            swerve.drive(new ChassisSpeeds());
        }, swerve);
    }

    public enum DifferentialType {
        TANK,
        ARCADE,
        CURVATURE
    }

    public enum CarType {
        FWD,
        RWD
    }
}