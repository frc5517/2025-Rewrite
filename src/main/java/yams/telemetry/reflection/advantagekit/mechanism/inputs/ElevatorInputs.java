package yams.telemetry.reflection.advantagekit.mechanism.inputs;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorInputs implements LoggableInputs
{

  /**
   * The {@link yams.mechanisms.positional.Elevator}s current height setpoint.
   */
  public Distance setpoint;

  @Override
  public void toLog(LogTable table)
  {
    table.put("setpoint", setpoint);
  }

  @Override
  public void fromLog(LogTable table)
  {
    setpoint = table.get("setpoint", setpoint);
  }
}
