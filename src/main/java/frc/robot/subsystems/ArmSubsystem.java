package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Telemetry;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
    private final DutyCycleEncoder armABS = new DutyCycleEncoder(Constants.ArmConstants.kArmABSID);
    private final SparkMax armMotor = new SparkMax(Constants.ArmConstants.kArmMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                    Constants.ArmConstants.kKp,
                    Constants.ArmConstants.kKi,
                    Constants.ArmConstants.kKd,
                    Constants.ArmConstants.kMaxSpeed,
                    Constants.ArmConstants.kMaxAcceleration)
            .withFeedforward(new ArmFeedforward(
                    Constants.ArmConstants.kS,
                    Constants.ArmConstants.kG,
                    Constants.ArmConstants.kV,
                    Constants.ArmConstants.kA))
            .withGearing(Constants.ArmConstants.kReduction)
//            .withExternalEncoder(armABS) Not yet supported current support is limited to attached encoders.
            .withSoftLimit(Constants.ArmConstants.kBottomSoftLimit, Constants.ArmConstants.kTopSoftLimit)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("Motor", Telemetry.armVerbosity)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Constants.ArmConstants.kRampRate)
            .withOpenLoopRampRate(Constants.ArmConstants.kRampRate)
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private SmartMotorController motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
    private ArmConfig armConfig = new ArmConfig(motor)
            .withLength(Constants.ArmConstants.kArmLength)
            .withHardLimit(Constants.ArmConstants.kBottomHardLimit, Constants.ArmConstants.kTopHardLimit)
            .withTelemetry("RobotTelemetry/Arm", Telemetry.armVerbosity)
            .withMass(Constants.ArmConstants.kArmMass)
            .withStartingPosition(Degrees.of(0))
            .withHorizontalZero(Constants.ArmConstants.kHorizontalZero);
    private Arm arm = new Arm(armConfig);

    public ArmSubsystem() {
        motor.setPosition(Degrees.of(armABS.get())); // Temp until rev through bores are supported
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }


    public Command goUp() {
        return arm.set(Constants.ArmConstants.kArmSpeed);
    }

    public Command goDown() {
        return arm.set(-Constants.ArmConstants.kArmSpeed);
    }

    public Command toL1() {
        return arm.setAngle(Constants.ArmConstants.kL1Setpoint);
    }

    public Command toL2() {
        return arm.setAngle(Constants.ArmConstants.kL2Setpoint);
    }

    public Command toL3() {
        return arm.setAngle(Constants.ArmConstants.kL3Setpoint);
    }

    public Command toL4() {
        return arm.setAngle(Constants.ArmConstants.kL4Setpoint);
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    public Command sysId() {
        return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }
}
