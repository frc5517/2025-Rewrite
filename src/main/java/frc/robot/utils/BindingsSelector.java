package frc.robot.utils;

public class BindingsSelector {

    public static BindingType selectedBinding = BindingType.SINGLE_XBOX;

    public enum BindingType {
        /*
        All controls using a single xbox controller.
         */
        SINGLE_XBOX,
        /*
        Classic driver and operator setup on two xbox controllers.
         */
        DUAL_XBOX,
        /*
        All controls on a single joystick.
         */
        SINGLE_STICK,
        /*
        All controls on two joysticks one driver.
         */
        DUAL_STICK,
        /*
        Classic driver and operator with the driver using a single joystick.
         */
        SINGLE_STICK_XBOX,
        /*
        Classic driver and operator with the driver using dual joysticks.
         */
        DUAL_STICK_XBOX,
        /*
        Control mode used for testing controls subject to constant change
         */
        TESTING
    }

}
