// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class Telemetry extends SubsystemBase {
    // Sets the verbosity level of robot telemetry.
    public static RobotTelemetry robotVerbosity = RobotTelemetry.HIGH;

    // Verbosity of other mechanisms.
    public static SwerveDriveTelemetry.TelemetryVerbosity swerveVerbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    public static SmartMotorControllerConfig.TelemetryVerbosity elevatorVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;
    public static SmartMotorControllerConfig.TelemetryVerbosity armVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;
    public static SmartMotorControllerConfig.TelemetryVerbosity intakeShooterVerbosity = SmartMotorControllerConfig.TelemetryVerbosity.HIGH;

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

    public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/RobotTelemetry/Sensors");

    // Drive Publishers
    // All drive telemetry is published courtesy of YAGSL!

    // Arm Publishers
    // All arm telemetry is published courtesy of YAMS!

    // Elevator Publishers
    // Most elevator telemetry is published courtesy of YAMS!
    public static final BooleanPublisher limitPublisher = table
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
}