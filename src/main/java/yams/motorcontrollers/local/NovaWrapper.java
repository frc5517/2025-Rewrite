package yams.motorcontrollers.local;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class NovaWrapper extends SmartMotorController
{

  @Override
  public void setupSimulation()
  {

  }

  @Override
  public void seedRelativeEncoder()
  {

  }

  @Override
  public void synchronizeRelativeEncoder()
  {

  }

  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {

  }

  @Override
  public void simIterate()
  {

  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {

  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {

  }

  @Override
  public void setEncoderPosition(Angle angle)
  {

  }

  @Override
  public void setEncoderPosition(Distance distance)
  {

  }

  @Override
  public void setPosition(Angle angle)
  {

  }

  @Override
  public void setPosition(Distance distance)
  {

  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {

  }

  @Override
  public void setVelocity(AngularVelocity angle)
  {

  }

  @Override
  public SysIdRoutine sysId(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    return null;
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    return false;
  }

  @Override
  public double getDutyCycle()
  {
    return 0;
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {

  }

  @Override
  public Current getSupplyCurrent()
  {
    return null;
  }

  @Override
  public Current getStatorCurrent()
  {
    return null;
  }

  @Override
  public Voltage getVoltage()
  {
    return null;
  }

  @Override
  public void setVoltage(Voltage voltage)
  {

  }

  @Override
  public DCMotor getDCMotor()
  {
    return null;
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return null;
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return null;
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    return null;
  }

  @Override
  public Angle getMechanismPosition()
  {
    return null;
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return null;
  }

  @Override
  public Angle getRotorPosition()
  {
    return null;
  }

  @Override
  public void updateTelemetry(NetworkTable table)
  {

  }

  @Override
  public void updateTelemetry()
  {

  }

  @Override
  public Temperature getTemperature()
  {
    return null;
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return null;
  }

  @Override
  public Object getMotorController()
  {
    return null;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    return null;
  }

  @Override
  public Optional<Angle> getMechanismSetpoint()
  {
    return Optional.empty();
  }
}
