package yams.gearing.gearbox;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Custom gearbox class
 */
public class CustomGearbox extends GearBox
{

  /**
   * Constructs a {@link GearBox} given the stages.
   *
   * @param stages Stages in the gear box.
   */
  public CustomGearbox(double[] stages)
  {
    super(stages);
  }

  /**
   * Constructs a {@link GearBox} given the stages.
   *
   * @param stages Stages in the gear box.
   */
  public CustomGearbox(String[] stages)
  {
    super(stages);
  }

  public boolean checkGearBoxLimit(double outputConversionFactor, double weight, DCMotor motorModel)
  {
    return true;
  }
}
