package yams.telemetry.reflection;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import yams.telemetry.reflection.advantagekit.mechanism.inputs.ElevatorInputs;
import yams.telemetry.reflection.advantagekit.motorcontroller.SmartMotorControllerInputs;

public class AdvantageData
{

  /**
   * {@link Map} containing the AdvantageKit inputs for the {@link Elevator} mechanisms.
   */
  Map<String, ElevatorInputs>             m_elevatorInputs = new HashMap<>();
  /**
   * {@link Map} containing the AdvantageKit inputs for the {@link SmartMotorController}s.
   */
  Map<String, SmartMotorControllerInputs> m_controllers    = new HashMap<>();

  /**
   * Record the inputs of a {@link SmartMotorController}
   *
   * @param telemetryName Telemetry name, usually MechanismTelemetryName/MotorControllerTelemetryName.
   * @param smc           {@link SmartMotorController} to record inputs of.
   */
  private void recordMechanismControllerInputs(String telemetryName, SmartMotorController smc)
  {
    if (!m_controllers.containsKey(telemetryName))
    {
      m_controllers.put(telemetryName, new SmartMotorControllerInputs());
    }
    SmartMotorControllerInputs controllerInputs = m_controllers.get(telemetryName);
    if (smc.getConfig().getSimpleClosedLoopController().isPresent())
    {
      PIDController pid = smc.getConfig().getSimpleClosedLoopController().get();
      controllerInputs.kp = pid.getP();
      controllerInputs.ki = pid.getI();
      controllerInputs.kd = pid.getD();
      controllerInputs.errorTolerance = Rotations.of(pid.getErrorTolerance());
      controllerInputs.errorDerivativeTolerance = Rotations.of(pid.getErrorDerivativeTolerance());
    } else if (smc.getConfig().getClosedLoopController().isPresent())
    {
      ProfiledPIDController pid = smc.getConfig().getClosedLoopController().get();
      controllerInputs.kp = pid.getP();
      controllerInputs.ki = pid.getI();
      controllerInputs.kd = pid.getD();
      controllerInputs.maxVelocity = RotationsPerSecond.of(pid.getConstraints().maxVelocity);
      controllerInputs.maxAcceleration = RotationsPerSecondPerSecond.of(pid.getConstraints().maxAcceleration);
    }
    if (smc.getConfig().getElevatorFeedforward().isPresent())
    {
      ElevatorFeedforward feedforward = smc.getConfig().getElevatorFeedforward().get();
      controllerInputs.ks = feedforward.getKs();
      controllerInputs.kv = feedforward.getKv();
      controllerInputs.ka = feedforward.getKa();
      controllerInputs.kg = feedforward.getKg();
      //feedforward.getDt()
    } else if (smc.getConfig().getArmFeedforward().isPresent())
    {
      ArmFeedforward feedforward = smc.getConfig().getArmFeedforward().get();
      controllerInputs.ks = feedforward.getKs();
      controllerInputs.kv = feedforward.getKv();
      controllerInputs.ka = feedforward.getKa();
      controllerInputs.kg = feedforward.getKg();
      //feedforward.getDt()
    } else if (smc.getConfig().getSimpleFeedforward().isPresent())
    {
      SimpleMotorFeedforward feedforward = smc.getConfig().getSimpleFeedforward().get();
      controllerInputs.ks = feedforward.getKs();
      controllerInputs.kv = feedforward.getKv();
      controllerInputs.ka = feedforward.getKa();
      //feedforward.getDt()
    }
    if (smc.getMechanismPositionSetpoint().isPresent())
    {
      controllerInputs.positionSetpoint = smc.getMechanismPositionSetpoint().get();
    }
    if (smc.getMechanismSetpointVelocity().isPresent())
    {
      controllerInputs.velocitySetpoint = smc.getMechanismSetpointVelocity().get();
    }
    Logger.processInputs(telemetryName, controllerInputs);
  }

  /**
   * Recording outputs of {@link SmartMotorController} for AdvantageKit
   *
   * @param telemetryName Base Telemetry name for the {@link SmartMotorController} in the form of
   *                      "MechanismTelemetryName/MotorControllerTelemetryName".
   * @param smc           {@link SmartMotorController} to use.
   */
  private void recordMotorControllerOutputs(String telemetryName, SmartMotorController smc)
  {
    Logger.recordOutput(telemetryName + "/measurement/position", smc.getMeasurementPosition());
    Logger.recordOutput(telemetryName + "/measurement/velocity", smc.getMeasurementVelocity());
    Logger.recordOutput(telemetryName + "/mechanism/position", smc.getMechanismPosition());
    Logger.recordOutput(telemetryName + "/mechanism/velocity", smc.getMechanismVelocity());
    Logger.recordOutput(telemetryName + "/rotor/position", smc.getRotorPosition());
    Logger.recordOutput(telemetryName + "/rotor/velocity", smc.getRotorVelocity());
    Logger.recordOutput(telemetryName + "/dutycycle", smc.getDutyCycle());
    Logger.recordOutput(telemetryName + "/current/stator", smc.getStatorCurrent());
    if (!(smc instanceof SparkWrapper))
    {
      Logger.recordOutput(telemetryName + "/current/supply", smc.getSupplyCurrent());
    }
    Logger.recordOutput(telemetryName + "/temperature", smc.getTemperature());
    Logger.recordOutput(telemetryName + "/voltage", smc.getVoltage());
  }

  /**
   * Record the outputs of the {@link Elevator} mechanism.
   *
   * @param elevator {@link Elevator} mechanism to record the outputs for.
   */
  public void recordMotorControllerOutputs(Elevator elevator)
  {
    SmartMotorController smc = elevator.getMotorController();
    if (elevator.getConfig().getTelemetryName().isPresent() &&
        smc.getConfig().getTelemetryName().isPresent())
    {
      String telemetryName = elevator.getConfig().getTelemetryName().get();
      Logger.recordOutput(telemetryName + "/height", elevator.getHeight());
      Logger.recordOutput(telemetryName + "/velocity", elevator.getVelocity());
      telemetryName += "/" + smc.getConfig().getTelemetryName().get();
      recordMotorControllerOutputs(telemetryName, smc);
    }
  }

  /**
   * Record the inputs for the {@link Elevator} mechanism within AdvantageKit.
   *
   * @param elevator {@link Elevator} mechanism.
   */
  public void recordMechanismControllerInputs(Elevator elevator)
  {
    SmartMotorController smc = elevator.getMotorController();
    if (elevator.getConfig().getTelemetryName().isPresent() &&
        smc.getConfig().getTelemetryName().isPresent())
    {
      String telemetryName = elevator.getConfig().getTelemetryName().get();
      if (!m_elevatorInputs.containsKey(telemetryName))
      {
        m_elevatorInputs.put(telemetryName, new ElevatorInputs());
      }
      ElevatorInputs elevatorInputs = m_elevatorInputs.get(telemetryName);
      elevatorInputs.setpoint = elevator.getMechanismSetpoint().isPresent() ? smc.getConfig()
                                                                                 .convertFromMechanism(elevator.getMechanismSetpoint()
                                                                                                               .get())
                                                                            : elevator.getHeight();
      Logger.processInputs(telemetryName, elevatorInputs);
      recordMechanismControllerInputs(telemetryName + "/" + smc.getConfig().getTelemetryName().get(), smc);
    }
  }
}
