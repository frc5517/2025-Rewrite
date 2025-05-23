// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class Telemetry extends SubsystemBase {
    // Elevator Publishers
    // Most elevator telemetry is published courtesy of YAMS!
    public static final BooleanPublisher limitPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Sensors")
            .getBooleanTopic("Elevator at bottom limit")
            .publish();
    // Intake Shooter Publishers
    public static final BooleanPublisher coralSensorPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Sensors")
            .getBooleanTopic("Coral in intake")
            .publish();
    public static final BooleanPublisher algaeSensorPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Sensors")
            .getBooleanTopic("Algae in intake")
            .publish();

    // Robot Container Publishers
    // SendableChoosers don't like this method
    // Pose selector publishers
    public static final StringPublisher selectedReefPosePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getStringTopic("Selected reef position")
            .publish();
    public static final StringPublisher selectedReefSidePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getStringTopic("Selected reef side")
            .publish();
    public static final StringPublisher selectedReefBranchPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getStringTopic("Selected reef branch")
            .publish();
    public static final StringPublisher selectedStationPosePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getStringTopic("Selected station position")
            .publish();
    public static final StringPublisher selectedStationSlotPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getStringTopic("Selected station slot")
            .publish();
    public static final StringPublisher selectedStationSidePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getStringTopic("Selected station side")
            .publish();
    public static final DoublePublisher selectedReefPoseCompassPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/RobotTelemetry/Pose Selection")
            .getDoubleTopic("Selected reef position as a compass")
            .publish();
    public static SmartMotorControllerConfig.TelemetryVerbosity armVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;
    public static SmartMotorControllerConfig.TelemetryVerbosity intakeShooterVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;
    // Sets the verbosity level of robot telemetry.
    public static RobotTelemetry robotVerbosity = RobotTelemetry.HIGH;
    // Verbosity of other mechanisms.
    public static SwerveDriveTelemetry.TelemetryVerbosity swerveVerbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    public static SmartMotorControllerConfig.TelemetryVerbosity elevatorVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;

    public enum RobotTelemetry {
        /*
         * No telemetry data is sent to dashboard.
         */
        NONE,

        /*
         * Only basic telemetry data is sent to dashboard.
         */
        LOW,

        /*
         * All telemetry data is sent to dashboard.
         */
        HIGH
    }

    // Drive Publishers
    // All drive telemetry is published courtesy of YAGSL!

    // Arm Publishers
    // All arm telemetry is published courtesy of YAMS!
}