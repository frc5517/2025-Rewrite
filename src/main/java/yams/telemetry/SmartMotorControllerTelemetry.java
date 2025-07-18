package yams.telemetry;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.networktables.NetworkTable;
import java.util.Map;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SmartMotorControllerTelemetry
{


  /**
   * {@link TelemetryVerbosity} for the {@link SmartMotorController}
   */
  private final TelemetryVerbosity                           verbosity = TelemetryVerbosity.HIGH;
  private       Map<DoubleTelemetryField, DoubleTelemetry>   doubleFields;
  private       Map<BooleanTelemetryField, BooleanTelemetry> boolFields;

  /**
   * Network table to publish to.
   */
  private NetworkTable                        dataNetworkTable;
  /**
   * Network table for tuning.
   */
  private NetworkTable                        tuningNetworkTable;
  /**
   * Telemetry config
   */
  private SmartMotorControllerTelemetryConfig config;

  /**
   * Setup Telemetry Pub/Sub fields.
   *
   * @param publishTable {@link NetworkTable} that holds all of the output fields.
   * @param tuningTable  {@link NetworkTable} that holds all of the tuning fields.
   * @param config       {@link SmartMotorControllerTelemetryConfig} to apply.
   */
  public void setupTelemetry(NetworkTable publishTable, NetworkTable tuningTable,
                             SmartMotorControllerTelemetryConfig config)
  {
    if (!publishTable.equals(this.dataNetworkTable))
    {
      dataNetworkTable = publishTable;
      this.config = config;
      doubleFields = config.getDoubleFields();
      boolFields = config.getBoolFields();
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }

    }
  }


  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable}
   */
  public void publish(SmartMotorController smc)
  {
    SmartMotorControllerConfig cfg = smc.getConfig();
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
    {
      BooleanTelemetry bt = entry.getValue();
      switch (bt.getField())
      {
        case MechanismUpperLimit ->
        {
          cfg.getMechanismUpperLimit().ifPresent(upperLimit -> bt.set(smc.getMechanismPosition().gte(upperLimit)));
        }
        case MechanismLowerLimit ->
        {
          cfg.getMechanismLowerLimit().ifPresent(lowerLimit -> bt.set(smc.getMechanismPosition().lte(lowerLimit)));
        }
        case TemperatureLimit ->
        {
          cfg.getTemperatureCutoff().ifPresent(temperatureCutoff -> bt.set(smc.getTemperature()
                                                                              .gte(temperatureCutoff)));
        }
        case VelocityControl ->
        {
          bt.set(smc.getMechanismSetpointVelocity().isPresent());
        }
        case ElevatorFeedForward ->
        {
          bt.set(cfg.getElevatorFeedforward().isPresent());
        }
        case ArmFeedForward ->
        {
          bt.set(cfg.getArmFeedforward().isPresent());
        }
        case SimpleMotorFeedForward ->
        {
          bt.set(cfg.getSimpleFeedforward().isPresent());
        }
        case MotionProfile ->
        {
          bt.set(cfg.getClosedLoopController().isPresent());
        }
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      switch (dt.getField())
      {
        case SetpointPosition ->
        {
          smc.getMechanismPositionSetpoint().ifPresent(mechSetpoint -> dt.set(
              cfg.getMechanismCircumference().isPresent() ? cfg.convertFromMechanism(mechSetpoint).in(Meters)
                                                          : mechSetpoint.in(Radians)));
        }
        case SetpointVelocity ->
        {
          smc.getMechanismSetpointVelocity().ifPresent(mechSetpoint ->
                                                           dt.set(cfg.getMechanismCircumference().isPresent() ?
                                                                  cfg.convertFromMechanism(mechSetpoint)
                                                                     .in(MetersPerSecond) :
                                                                  mechSetpoint.in(RadiansPerSecond)));
        }
        case FeedforwardVoltage ->
        {
          // TODO: Remove
//          smc.getFeedforwardVoltage();
        }
        case PIDOutputVoltage ->
        {
          // TODO: Remove
//          smc.getPIDOutputVoltage();
        }
        case OutputVoltage ->
        {
          dt.set(smc.getVoltage().in(Volts));
        }
        case StatorCurrent ->
        {
          dt.set(smc.getStatorCurrent().in(Amps));
        }
        case SupplyCurrent ->
        {
          smc.getSupplyCurrent().ifPresent(current -> dt.set(current.in(Amps)));
        }
        case MotorTemperature ->
        {
          dt.set(smc.getTemperature().in(Fahrenheit));
        }
        case MeasurementPosition ->
        {
          cfg.getMechanismCircumference().ifPresent(circumference -> dt.set(smc.getMeasurementPosition().in(Meters)));
        }
        case MeasurementVelocity ->
        {
          cfg.getMechanismCircumference().ifPresent(circumference -> dt.set(smc.getMeasurementVelocity()
                                                                               .in(MetersPerSecond)));
        }
        case MechanismPosition ->
        {
          dt.set(smc.getMechanismPosition().in(Radians));
        }
        case MechanismVelocity ->
        {
          dt.set(smc.getMechanismVelocity().in(RadiansPerSecond));
        }
        case RotorPosition ->
        {
          dt.set(smc.getRotorPosition().in(Radians));
        }
        case RotorVelocity ->
        {
          dt.set(smc.getRotorVelocity().in(RadiansPerSecond));
        }
      }
    }
  }


  public void applyTuningValues(SmartMotorController smartMotorController)
  {

    SmartMotorControllerConfig cfg = smartMotorController.getConfig();
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      if (dt.tunable())
      {
        switch (dt.getField())
        {
          case SetpointPosition ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(circumference -> smartMotorController.setPosition(Meters.of(
                dt.get())), () -> smartMotorController.setPosition(Radians.of(dt.get())));
          }
          case SetpointVelocity ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(circumference -> smartMotorController.setVelocity(
                MetersPerSecond.of(
                    dt.get())), () -> smartMotorController.setVelocity(RadiansPerSecond.of(dt.get())));
          }
        }
      }
    }
  }

  // TODO: Add docs.
  public enum BooleanTelemetryField
  {
    MechanismUpperLimit("limits/Mechanism Upper Limit", false, false),
    MechanismLowerLimit("limits/Mechanism Lower Limit", false, false),
    TemperatureLimit("limits/Temperature Limit", false, false),
    VelocityControl("control/Velocity Control", false, false),
    ElevatorFeedForward("control/Elevator Feedforward", false, false),
    ArmFeedForward("control/Arm Feedforward", false, false),
    SimpleMotorFeedForward("control/Simple Motor Feedforward", false, false),
    MotionProfile("control/Motion Profile", false, false);

    private final boolean currentValue;
    private final String  key;
    private final boolean tunable;

    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable)
    {
      key = fieldName;
      currentValue = defaultValue;
      this.tunable = tunable;
    }

    public BooleanTelemetry create()
    {
      return new BooleanTelemetry(key, currentValue, this, tunable);
    }


  }

  public enum DoubleTelemetryField
  {
    SetpointPosition("Setpoint Position", 0, true),
    SetpointVelocity("Setpoint Velocity", 0, false),
    FeedforwardVoltage("Feedforward voltage", 0, false),
    PIDOutputVoltage("PID Output voltage", 0, false),
    OutputVoltage("Output Voltage", 0, false),
    StatorCurrent("Stator Current", 0, false),
    SupplyCurrent("Supply Current", 0, false),
    MotorTemperature("Motor Temperature", 0, false),
    MeasurementPosition("Measurement Position", 0, false),
    MeasurementVelocity("Measurement Velocity", 0, false),
    MechanismPosition("Mechanism Position", 0, false),
    MechanismVelocity("Mechanism Velocity", 0, false),
    RotorPosition("Rotor Position", 0, false),
    RotorVelocity("Rotor Velocity", 0, false);

    private final double  defaultVal;
    private final String  key;
    private final boolean tunable;

    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable)
    {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    public DoubleTelemetry create()
    {
      return new DoubleTelemetry(key, defaultVal, this, tunable);
    }

  }
}
