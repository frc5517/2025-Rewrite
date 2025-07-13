package yams.telemetry;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import java.util.Optional;

public class MechanismTelemetry
{

  /**
   * Telemetry NetworkTable.
   */
  public Optional<NetworkTable> ntable = Optional.empty();
  /**
   * Network Tables publisher for the units displayed by the positional mechanism.
   */
  public StringPublisher        units;
  /**
   * Network Tables Publisher for the setpoint.
   */
  public DoublePublisher        setpointPublisher;
  /**
   * Network Tables publisher for the position of the mechanism.
   */
  public DoublePublisher        positionPublisher;

  /**
   * Setup telemetry for the Mechanism
   *
   * @param table NetworkTable to publish to.
   */
  public void setupTelemetry(NetworkTable table)
  {
    if (table != null)
    {
      this.ntable = Optional.of(table);
      units = table.getStringTopic("Units").publish();
      setpointPublisher = table.getDoubleTopic("Setpoint").publish();
      positionPublisher = table.getDoubleTopic("Position").publish();
    }
  }
}
