package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Telemetry;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.*;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.Telemetry.limitPublisher;
import static yams.mechanisms.SmartMechanism.*;

public class Elevator extends SubsystemBase {
    /**
     * The setpoint constants for this elevator subsystem.
     */
    public static final class ControlConstants {
        public static final double                  kElevatorSpeed =                .4; // Used elsewhere to run armCMD at this DutyCycle
        public static final Distance                kAtHeightTolerance =            Inches.of(0.5); // How far from the angle is acceptable.

        public static final Distance                kL1Setpoint =                   Inches.of(10);
        public static final Distance                kL2Setpoint =                   Inches.of(6);
        public static final Distance                kL3Setpoint =                   Inches.of(25);
        public static final Distance                kL4Setpoint =                   Inches.of(60);
        public static final Distance                kProcessorSetpoint =            Inches.of(2);
        public static final Distance                kStationSetpoint =              Inches.of(1);
        public static final Distance                kDealgaeHigh =                  Inches.of(37.5);
        public static final Distance                kDealgaeLow =                   Inches.of(20);
        public static final Distance                kStowSetpoint =                 Inches.of(0);
    }
    /**
     * The hardware constants for this elevator subsystem.
     */
    public static final class HardwareConstants {
        public static final int                     kLeadMotorID =                 14; // Lead motor is typically left for CCW+ with no inversion.
        public static final boolean                 kLeadMotorInversion =          false;
        public static final int                     kFollowerMotorID =              13;
        public static final boolean                 kFollowerMotorInversion =       true;
        public static final int                     kBottomLimitPort =              2;

        public static final double                  kBottomCarriageToArmInches =    32;
        public static final double                  kCenterToElevator =             Units.inchesToMeters(10);

        public static final Distance                kTopSoftLimit =                 Meters.of(Units.inchesToMeters(63.5));
        public static final Distance                kBottomSoftLimit =              Meters.of(Units.inchesToMeters(0.5));
        public static final Distance                kTopHardLimit =                 Meters.of(Units.inchesToMeters(64));
        public static final Distance                kBottomHardLimit =              Meters.of(Units.inchesToMeters(0));
        public static final Mass                    kMass =                         Kilograms.of(Units.lbsToKilograms(10));
        public static final MechanismGearing        kReduction =                    gearing(gearbox(3, 5), sprocket(22 / 22.0));
        public static final Distance                kMechanismCircumference =       Meters.of(Inches.of(0.25).in(Meters) * 22);
        public static final Time                    kClosedLoopRampRate =           Seconds.of(0.25);
        public static final Time                    kOpenLoopRampRate =             Seconds.of(0.25);
        public static final Current                 kStatorLimit =                  Amp.of(40);

        public static final class kProfiledPID {
            public static final double              kKp =                           12.0;
            public static final double              kKi =                           0.0;
            public static final double              kKd =                           0.0;
            public static final LinearVelocity      kMaxVelocity =                  MetersPerSecond.of(2);
            public static final LinearAcceleration  kMaxAcceleration =              MetersPerSecondPerSecond.of(3);
        }
        public static final ElevatorFeedforward     kFF =                           new ElevatorFeedforward(
                0,
                0,
                0,
                0);
        public static final ControlMode             kControlMode =                  ControlMode.CLOSED_LOOP;
        public static final TelemetryVerbosity      kVerbosity =                    TelemetryVerbosity.HIGH;
    }
    /**
     * The constants used to simulate the elevator, and it's 3D location.
     */
    public static final class SimConstants {
        public static final Distance                kSimStartingHeight =            Inches.of(0);
        public static final Distance                kMaxRobotHeight =               Inches.of(70);
        public static final Distance                kMaxRobotLength =               Inches.of(36);
        /**
         * Values are from robot center.
         * Look at WPI Coordinate System if unsure.
         */
        public static final class kMechanismPosition {
            public static final Distance            kXFrontPositive =               Inches.of(0);
            public static final Distance            kYLeftPositive =                Inches.of(0.0);
            public static final Distance            kZUpPositive =                  Inches.of(12);
        }
    }
    /**
     * A bottoming limit switch to prevent the elevator from continuing to drive past physical limits.
     */
    private final DigitalInput limitSwitch = new DigitalInput(HardwareConstants.kBottomLimitPort);
    /**
     * The lead motor driving the elevator mechanism.
     */
    private final SparkMax elevatorRightMotor = new SparkMax(HardwareConstants.kLeadMotorID, SparkLowLevel.MotorType.kBrushless);
    /**
     * The follower motor driving the elevator mechanism.
     */
    private final SparkMax elevatorLeftMotor = new SparkMax(HardwareConstants.kFollowerMotorID, SparkLowLevel.MotorType.kBrushless);
    /**
     * The config to apply to the motor driving the elevator.
     */
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withMechanismCircumference(HardwareConstants.kMechanismCircumference)
            .withClosedLoopController(HardwareConstants.kProfiledPID.kKp, HardwareConstants.kProfiledPID.kKi, HardwareConstants.kProfiledPID.kKd,
                    HardwareConstants.kProfiledPID.kMaxVelocity, HardwareConstants.kProfiledPID.kMaxAcceleration)
            .withSoftLimit(HardwareConstants.kBottomSoftLimit, HardwareConstants.kTopSoftLimit)
            .withGearing(HardwareConstants.kReduction)
//      .withExternalEncoder(elevatorMotor.getAbsoluteEncoder())
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("ElevatorMotor", HardwareConstants.kVerbosity)
            .withStatorCurrentLimit(HardwareConstants.kStatorLimit)
            .withMotorInverted(HardwareConstants.kLeadMotorInversion)
            .withClosedLoopRampRate(HardwareConstants.kClosedLoopRampRate)
            .withOpenLoopRampRate(HardwareConstants.kOpenLoopRampRate)
            .withFeedforward(HardwareConstants.kFF)
            .withControlMode(HardwareConstants.kControlMode)
            .withFollowers(Pair.of(elevatorRightMotor, HardwareConstants.kFollowerMotorInversion)); // Needs changed if follower is removed.
    /**
     * Initialize the wrapper with the driving motor and it's config.
     */
    private final SmartMotorController motor = new SparkWrapper(
            elevatorLeftMotor,
            DCMotor.getNEO(2), // Needs changed if follower is removed.
            motorConfig);
    /**
     * This config is used to place the simulated mechanism on a simulated robot.
     */
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
            .withMaxRobotHeight(SimConstants.kMaxRobotHeight)
            .withMaxRobotLength(SimConstants.kMaxRobotLength)
            .withRelativePosition(new Translation3d(
                    SimConstants.kMechanismPosition.kXFrontPositive,
                    SimConstants.kMechanismPosition.kYLeftPositive,
                    SimConstants.kMechanismPosition.kZUpPositive));
    /**
     * The config to apply to the elevator mechanism.
     */
    private final ElevatorConfig m_config = new ElevatorConfig(motor)
            .withStartingHeight(SimConstants.kSimStartingHeight)
            .withHardLimits(HardwareConstants.kBottomHardLimit, HardwareConstants.kTopHardLimit)
            .withTelemetry("Elevator", HardwareConstants.kVerbosity)
            .withMechanismPositionConfig(robotToMechanism)
            .withMass(HardwareConstants.kMass);
    /**
     * Initialize the final elevator mechanism with the {@link ElevatorConfig}.
     */
    private final yams.mechanisms.positional.Elevator elevator = new yams.mechanisms.positional.Elevator(m_config);

    public Elevator() {
        Trigger atBottomSwitch = new Trigger(limitSwitch::get);
        atBottomSwitch.onTrue(
                elevator.set(0)
                        .andThen(() -> motor.setPosition(Meters.of(0))));
        elevator.getMechanismLigament().setColor(new Color8Bit(Color.kPurple));
    }

    public yams.mechanisms.positional.Elevator getElevator() {
        return elevator;
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Sends custom robot telemetry if verbosity is high enough.
        if (Telemetry.robotVerbosity.ordinal() >= Telemetry.RobotTelemetry.LOW.ordinal()) {
            limitPublisher.set(limitSwitch.get());
        }
        // Updates the elevator mechanism's telemetry data to the network tables.
        elevator.updateTelemetry();
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        elevator.simIterate();
    }

    /**
     * Set the DutyCycle of the {@link yams.motorcontrollers.SmartMotorController}.
     *
     * @param dutyCycle [-1,1] to set.
     * @return {@link Command}
     */
    public Command elevCmd(double dutyCycle) {
        return elevator.set(dutyCycle);
    }

    /**
     * Set the height of the elevator.
     *
     * @param height Height of the elevator to reach.
     * @return {@link Command} that sets the elevator height, stops immediately.
     */
    public Command setHeight(Distance height) {
        return elevator.setHeight(height);
    }

    /**
     * Checks if the elevator is near another distance.
     * Tolerance is specified in {@link Elevator.ControlConstants}
     *
     * @return true if the elevator is near another distance, otherwise false.
     */
    public boolean atHeight(Distance height) {
        return elevator.getHeight().isNear(height, ControlConstants.kAtHeightTolerance);
    }

    /**
     * Get the Height of the Elevator.
     *
     * @return {@link Distance} of the Elevator.
     */
    public Distance getHeight() {
        return elevator.getHeight();
    }

    /**
     * Runs a sysID test.
     * @return a {@link Command} that runs a sysID test.
     */
    public Command sysId() {
        return elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
    }
}

