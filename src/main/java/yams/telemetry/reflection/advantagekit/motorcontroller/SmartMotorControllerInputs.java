package yams.telemetry.reflection.advantagekit.motorcontroller;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SmartMotorControllerInputs implements LoggableInputs
{

  /**
   * Feedback gains.
   */
  public double kp = 0, ki = 0, kd = 0;
  /**
   * Feedforward gains.
   */
  public double ks = 0, kv = 0, ka = 0, kg = 0;
  /**
   * Maximum Velocity of the closed loop controller.
   */
  public AngularVelocity     maxVelocity              = RotationsPerSecond.of(0);
  /**
   * Maximum Acceleration of the closed loop controller.
   */
  public AngularAcceleration maxAcceleration          = RotationsPerSecondPerSecond.of(0);
  /**
   * Motor controller error tolerance.
   */
  public Angle               errorTolerance           = Rotations.of(0);
  /**
   * Motor controller error derivative tolerance.
   */
  public Angle               errorDerivativeTolerance = Rotations.of(0);
  /**
   * Motor controller position setpoint.
   */
  public Angle           positionSetpoint = Rotations.of(0);
  /**
   * Motor controller velocity setpoint.
   */
  public AngularVelocity velocitySetpoint = RotationsPerSecond.of(0);

  @Override
  public void toLog(LogTable table)
  {
    table.put("kp", kp);
    table.put("ki", ki);
    table.put("kd", kd);
    table.put("ks", ks);
    table.put("kv", kv);
    table.put("ka", ka);
    table.put("kg", kg);
    table.put("errorTolerance", errorTolerance);
    table.put("errorDerivativeTolerance", errorDerivativeTolerance);
    table.put("maxVelocity", maxVelocity);
    table.put("maxAcceleration", maxAcceleration);
    table.put("velocitySetpoint", velocitySetpoint);
    table.put("positionSetpoint", positionSetpoint);
  }

  @Override
  public void fromLog(LogTable table)
  {
    kp = table.get("kp", kp);
    ki = table.get("ki", ki);
    kd = table.get("kd", kd);
    ks = table.get("ks", ks);
    kv = table.get("kv", kv);
    ka = table.get("ka", ka);
    kg = table.get("kg", kg);
    errorTolerance = table.get("errorTolerance", errorTolerance);
    errorDerivativeTolerance = table.get("errorDerivativeTolerance", errorTolerance);
    maxVelocity = table.get("maxVelocity", maxVelocity);
    maxAcceleration = table.get("maxAcceleration", maxAcceleration);
    positionSetpoint = table.get("positionSetpoint", positionSetpoint);
    velocitySetpoint = table.get("velocitySetpoint", velocitySetpoint);
  }
}
