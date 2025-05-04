package yams.gearing.gearbox;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * MAXPlanetary gearbox class
 */
public class MAXPlanetaryGearbox extends GearBox
{

  /**
   * Constructs a {@link GearBox} given the stages.
   *
   * @param stages Stages in the gear box.
   */
  public MAXPlanetaryGearbox(double[] stages)
  {
    super(stages);
  }

  /**
   * Constructs a {@link GearBox} given the stages.
   *
   * @param stages Stages in the gear box.
   */
  public MAXPlanetaryGearbox(String[] stages)
  {
    super(stages);
  }

  public boolean checkGearBoxLimit(double outputConversionFactor, double weight, DCMotor motorModel)
  {
    return true;
  }
}
