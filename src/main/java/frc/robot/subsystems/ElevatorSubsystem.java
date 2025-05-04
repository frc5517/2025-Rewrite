package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax elevatorRightMotor = new SparkMax(Constants.ElevatorConstants.kRightMotorID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax elevatorLeftMotor = new SparkMax(Constants.ElevatorConstants.kLeftMotorID, SparkLowLevel.MotorType.kBrushless);
    private SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withMechanismCircumference(Constants.ElevatorConstants.kSprocketCircumference)
            .withClosedLoopController(
                    Constants.ElevatorConstants.kKp,
                    Constants.ElevatorConstants.kKi,
                    Constants.ElevatorConstants.kKd,
                    Constants.ElevatorConstants.kMaxSpeed,
                    Constants.ElevatorConstants.kMaxAcceleration)
            .withFeedforward(new ElevatorFeedforward(
                    Constants.ElevatorConstants.kS,
                    Constants.ElevatorConstants.kG,
                    Constants.ElevatorConstants.kV,
                    Constants.ElevatorConstants.kA))
            .withSoftLimit(Constants.ElevatorConstants.kBottomSoftLimit, Constants.ElevatorConstants.kTopSoftLimit)
            .withGearing(Constants.ElevatorConstants.kReduction)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("ElevatorMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withVoltageCompensation(Volts.of(12))
            .withMotorInverted(true)
            .withClosedLoopRampRate(Constants.ElevatorConstants.kRampRate)
            .withOpenLoopRampRate(Constants.ElevatorConstants.kRampRate)
            .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
            .withFollowers(Pair.of(elevatorLeftMotor, false));
    private SmartMotorController motor = new SparkWrapper(elevatorRightMotor, DCMotor.getNEO(2), motorConfig);
    private ElevatorConfig elevatorConfig = new ElevatorConfig(motor)
            .withStartingHeight(Meters.of(0.5))
            .withHardLimits(Constants.ElevatorConstants.kBottomHardLimit, Constants.ElevatorConstants.kTopHardLimit)
            .withTelemetry("Elevator", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withMass(Constants.ElevatorConstants.kMass);
    private Elevator elevator = new Elevator(elevatorConfig);

    private DigitalInput limitSwitch = new DigitalInput(Constants.ElevatorConstants.kBottomLimitPort);
    private Trigger atBottomSwitch = new Trigger(() -> limitSwitch.get());

    public ElevatorSubsystem()
    {
        atBottomSwitch.onTrue(
                elevator.set(0)
                        .andThen(() -> motor.setPosition(Meters.of(0))));
    }

    @Override
    public void periodic()
    {
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic()
    {
        elevator.simIterate();
    }

    public Command toL1()
    {
        return elevator.setHeight(Constants.ElevatorConstants.kL1Setpoint);
    }

    public Distance getHeight() {
        return elevator.getHeight();
    }
}

