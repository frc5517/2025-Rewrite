package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Telemetry;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.Telemetry.limitPublisher;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorRightMotor = new SparkMax(ElevatorConstants.kRightMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax elevatorLeftMotor = new SparkMax(ElevatorConstants.kLeftMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withMechanismCircumference(ElevatorConstants.kSprocketCircumference)
            .withClosedLoopController(4, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(3))
            .withSoftLimit(ElevatorConstants.kBottomSoftLimit, ElevatorConstants.kTopSoftLimit)
            .withGearing(ElevatorConstants.kReduction)
//      .withExternalEncoder(elevatorMotor.getAbsoluteEncoder())
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("ElevatorMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
            .withFollowers(Pair.of(elevatorRightMotor, true));
    private final SmartMotorController motor = new SparkWrapper(elevatorLeftMotor,
            DCMotor.getNEO(2),
            motorConfig);
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
    private final ElevatorConfig m_config = new ElevatorConfig(motor)
            .withStartingHeight(Inches.of(1))
            .withHardLimits(ElevatorConstants.kBottomHardLimit, ElevatorConstants.kTopHardLimit)
            .withTelemetry("Elevator", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withMechanismPositionConfig(robotToMechanism)
            .withMass(ElevatorConstants.kMass);
    private final yams.mechanisms.positional.Elevator elevator = new yams.mechanisms.positional.Elevator(m_config);
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitPort);

    public Elevator() {
        Trigger atBottomSwitch = new Trigger(limitSwitch::get);
        atBottomSwitch.onTrue(
                elevator.set(0)
                        .andThen(() -> motor.setPosition(Meters.of(0))));
    }

    @Override
    public void periodic() {
        if (Telemetry.robotVerbosity.ordinal() >= Telemetry.RobotTelemetry.LOW.ordinal()) {
            limitPublisher.set(limitSwitch.get());
        }
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }

    public Command elevCmd(double dutycycle) {
        return elevator.set(dutycycle);
    }

    public Distance getHeight() {
        return elevator.getHeight();
    }

    public Command setHeight(Distance height) {
        return elevator.setHeight(height);
    }

    public boolean atHeight(Distance height) {
        return elevator.getHeight() == height;
    }

    public Command sysId() {
        return elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
    }
}

