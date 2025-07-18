package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;

import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax                   armMotor    = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSoftLimit(ArmConstants.kBottomSoftLimit, ArmConstants.kTopSoftLimit)
            .withGearing(ArmConstants.kReduction)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("ArmMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorController       motor       = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
    private final MechanismPositionConfig    robotToMechanism = new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));
    private final ArmConfig                  m_config    = new ArmConfig(motor)
            .withLength(ArmConstants.kArmLength)
            .withHardLimit(ArmConstants.kBottomHardLimit, ArmConstants.kTopHardLimit)
            .withTelemetry("Arm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withMass(ArmConstants.kArmMass)
            .withStartingPosition(Degrees.of(0))
            .withHorizontalZero(Degrees.of(0))
            .withMechanismPositionConfig(robotToMechanism);
    private final Arm                        arm         = new Arm(m_config);

    public ArmSubsystem()
    {
        //motor.setPosition(Degrees.of(armABS.get()));
    }

    public void periodic()
    {
        arm.updateTelemetry();
    }

    public void simulationPeriodic()
    {
        arm.simIterate();
    }

    public Command armCmd(double dutycycle)
    {
        return arm.set(dutycycle)
                .finallyDo(() -> arm.set(0.0));
    }

    public Command sysId()
    {
        return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    public Command setAngle(Angle angle)
    {
        return arm.setAngle(angle);
    }

    public boolean atAngle(Angle angle) {
        return arm.getAngle() == angle;
    }
}