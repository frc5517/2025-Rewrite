package yams.gearing.gearbox;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Versaplanetary gearbox
 */
public class VersaPlanetaryGearBox extends GearBox
{

  /**
   * Constructs an {@link GearBox} given the stages.
   *
   * @param stage Stages in the gear box.
   */
  public VersaPlanetaryGearBox(double[] stage)
  {
    super(stage);
  }

  /**
   * Constructs an {@link GearBox} given the stages.
   *
   * @param stage Stages in the gear box.
   */
  public VersaPlanetaryGearBox(String[] stage)
  {
    super(stage);
  }

  public boolean checkGearBoxLimit(double outputConversionFactor, double weight, DCMotor motorModel)
  {
    return true;
  }
}
