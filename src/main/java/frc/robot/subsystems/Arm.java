package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.*;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static yams.mechanisms.SmartMechanism.*;

public class Arm extends SubsystemBase {
    /**
     * The setpoint constants for this arm subsystem.
     */
    public static final class ControlConstants {
        public static final double                  kArmSpeed =                     0.3; // Used elsewhere to run armCMD at this DutyCycle
        public static final Angle                   kAtAngleTolerance =             Degrees.of(1);

        public static final Angle                   kL1Setpoint =                   Degrees.of(-20);
        public static final Angle                   kL2Setpoint =                   Degrees.of(-20);
        public static final Angle                   kL3Setpoint =                   Degrees.of(-20);
        public static final Angle                   kL4Setpoint =                   Degrees.of(-35);
        public static final Angle                   kProcessorSetpoint =            Degrees.of(0);
        public static final Angle                   kStationSetpoint =              Degrees.of(32);
        public static final Angle                   kDealgae =                      Degrees.of(-15);
        public static final Angle                   kStowSetpoint =                 Degrees.of(70);
    }
    /**
     * The hardware constants for this arm subsystem.
     */
    public static final class HardwareConstants {
        public static final int                     kArmMotorID =                   12;
        public static final boolean                 kArmMotorInversion =            false;
        public static final int                     kArmABSID =                     1; // DIO

        public static final Angle                   kTopSoftLimit =                 Degrees.of(74);
        public static final Angle                   kBottomSoftLimit =              Degrees.of(-89);
        public static final Angle                   kTopHardLimit =                 Degrees.of(75);
        public static final Angle                   kBottomHardLimit =              Degrees.of(-90);
        public static final Mass                    kArmMass =                      Pounds.of(3);
        public static final Distance                kArmLength =                    Inches.of(17);
        public static final Angle                   kHorizontalZero =               Degrees.of(0); // -173
        public static final MechanismGearing        kReduction =                    gearing(gearbox(3, 4, 5), sprocket(16 / 38.0));
        public static final Time                    kClosedLoopRampRate =           Seconds.of(0.25);
        public static final Time                    kOpenLoopRampRate =             Seconds.of(0.25);
        public static final Current                 kStatorLimit =                  Amp.of(40);

        public static final class kProfiledPID {
            public static final double              kKp =                           8.0;
            public static final double              kKi =                           0.0;
            public static final double              kKd =                           0.0;
            public static final AngularVelocity     kMaxVelocity =                  DegreesPerSecond.of(180);
            public static final AngularAcceleration kMaxAcceleration =              DegreesPerSecondPerSecond.of(360);
        }
        public static final ArmFeedforward          kFF =                           new ArmFeedforward(
                0,
                0,
                0,
                0);
        public static final MotorMode               kMotorMode =                    MotorMode.BRAKE;
        public static final ControlMode             kControlMode =                  ControlMode.CLOSED_LOOP;
        public static final TelemetryVerbosity      kVerbosity =                    TelemetryVerbosity.HIGH;
    }

    /**
     * The constants used to simulate the arm, and it's 3D location.
     */
    public static final class SimConstants {
        public static final Angle                   kSimStartingAngle =             Degrees.of(0);
        public static final Distance                kMaxRobotHeight =               Meters.of(1.5);
        public static final Distance                kMaxRobotLength =               Meters.of(0.75);
        public static final Distance                kWindowCenter =                 Meters.of(3);
        /**
         * Values not from robot center.
         * Look at WPI Coordinate System if unsure.
         */
        public static final class kMechanismPosition {
            public static final Distance            kXFrontPositive =               Inches.of(60);
            public static final Distance            kYLeftPositive =                Inches.of(0.0);
            public static final Distance            kZUpPositive =                  Inches.of(20);
        }
    }
    /**
     * The motor driving the arm mechanism.
     */
    private final SparkMax armMotor = new SparkMax(HardwareConstants.kArmMotorID, SparkLowLevel.MotorType.kBrushless);
    /**
     * The config to apply to the motor driving the arm.
     */
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(HardwareConstants.kProfiledPID.kKp, HardwareConstants.kProfiledPID.kKi, HardwareConstants.kProfiledPID.kKd,
                    HardwareConstants.kProfiledPID.kMaxVelocity, HardwareConstants.kProfiledPID.kMaxAcceleration)
            .withSoftLimit(HardwareConstants.kBottomSoftLimit, HardwareConstants.kTopSoftLimit)
            .withGearing(HardwareConstants.kReduction)
            .withIdleMode(HardwareConstants.kMotorMode)
            .withTelemetry("ArmMotor", HardwareConstants.kVerbosity)
            .withStatorCurrentLimit(HardwareConstants.kStatorLimit)
            .withMotorInverted(HardwareConstants.kArmMotorInversion)
            .withClosedLoopRampRate(HardwareConstants.kClosedLoopRampRate)
            .withOpenLoopRampRate(HardwareConstants.kOpenLoopRampRate)
            .withFeedforward(HardwareConstants.kFF)
            .withControlMode(HardwareConstants.kControlMode);
    /**
     * Initialize the wrapper with the driving motor and it's config.
     */
    private final SmartMotorController motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
    /**
     * This config is used to place the simulated mechanism on a simulated robot.
     */
    public final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
            .withMaxRobotHeight(SimConstants.kMaxRobotHeight)
            .withMaxRobotLength(SimConstants.kMaxRobotLength)
            .withRelativePosition(new Translation3d(
                    SimConstants.kMechanismPosition.kXFrontPositive,
                    SimConstants.kMechanismPosition.kYLeftPositive,
                    SimConstants.kMechanismPosition.kZUpPositive));
    /**
     * The config to apply to the arm mechanism.
     */
    private final ArmConfig m_config = new ArmConfig(motor)
            .withLength(HardwareConstants.kArmLength)
            .withHardLimit(HardwareConstants.kBottomHardLimit, HardwareConstants.kTopHardLimit)
            .withTelemetry("Arm", HardwareConstants.kVerbosity)
            .withMass(HardwareConstants.kArmMass)
            .withStartingPosition(SimConstants.kSimStartingAngle)
            .withHorizontalZero(HardwareConstants.kHorizontalZero)
            .withMechanismPositionConfig(robotToMechanism);
    /**
     * Initialize the final arm mechanism with the {@link ArmConfig}.
     */
    private final yams.mechanisms.positional.Arm arm = new yams.mechanisms.positional.Arm(m_config);

    /**
     * Called when this class is initialized.
     */
    public Arm() {
        //motor.setPosition(Degrees.of(armABS.get()));
    }

    public yams.mechanisms.positional.Arm getArm() {
        return arm;
    }

    /**
     * Ran continuously while the robot is on.
     */
    @Override
    public void periodic() {
        // Updates the arm mechanism's telemetry data to the network tables.
        arm.updateTelemetry();
    }

    /**
     * Ran continuously when the robot is in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Iterates the sim so that the sim actually works and the data sent to the network tables can be updated.
        arm.simIterate();
    }

    /**
     * Set the DutyCycle of the {@link yams.motorcontrollers.SmartMotorController}.
     *
     * @param dutycycle [-1,1] to set.
     * @return {@link Command}
     */
    public Command armCmd(double dutycycle) {
        return arm.set(dutycycle)
                .finallyDo(() -> arm.set(0.0));
    }

    /**
     * Set the arm to the given angle.
     *
     * @param angle Arm angle to go to.
     * @return {@link Command} that sets the arm to the desired angle.
     */
    public Command setAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    /**
     * Checks if the arm is near another angle.
     * Tolerance is specified in {@link ControlConstants}
     *
     * @return true if the arm is near another angle, otherwise false.
     */
    public boolean atAngle(Angle angle) {
        return arm.getAngle().isNear(angle, ControlConstants.kAtAngleTolerance);
    }

    /**
     * Get the {@link SmartMotorController} Mechanism Position representing the arm.
     *
     * @return Arm {@link Angle}
     */
    public Angle getAngle() {
        return arm.getAngle();
    }

    /**
     * Runs a sysID test.
     * @return a {@link Command} that runs a sysID test.
     */
    public Command sysId() {
        return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }
}