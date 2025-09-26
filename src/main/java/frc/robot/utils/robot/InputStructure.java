package frc.robot.utils.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * RobotControlBindings - Centralized control bindings management for robot operation.
 * <p>
 * This class manages all control input bindings for different operator configurations.
 * Each binding method contains its own speed constants to allow for easy customization
 * when creating driver-specific control schemes.
 * <p>
 * Available control schemes:
 * - Single operator with Xbox controller
 * - Dual operator with two Xbox controllers
 * - Single operator with Logitech Extreme 3D Pro joystick
 * - Dual operator with two Logitech Extreme 3D Pro joysticks
 * - Mixed configurations (stick + Xbox)
 * - Test mode bindings
 * <p>
 * The class uses a mode-based binding system where only the selected control scheme
 * is active at any given time, preventing control conflicts between different input devices.
 */
public class InputStructure {

    // Control chooser for dashboard
    private final SendableChooser<BindingType> controlChooser = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;
    private final CommandJoystick driverRightStick;  // Flight Stick
    private final CommandJoystick driverLeftStick; // Flight Stick again
    // Robot subsystems
    private final SwerveSubsystem swerve;
    private final Elevator elevator;
    private final Arm arm;
    private final IntakeShooter intakeShooter;
    private final ControlStructure structure;
    private final PoseSelector poseSelector;
    /**
     * Constructs a new RobotControlBindings instance.
     *
     * @param swerve        Swerve drivetrain subsystem
     * @param elevator      Elevator subsystem
     * @param arm           Arm subsystem
     * @param intakeShooter Intake/shooter subsystem
     * @param structure     Control structure for auto commands
     * @param poseSelector  Pose selection system
     */
    public InputStructure(
            SwerveSubsystem swerve,
            Elevator elevator,
            Arm arm,
            IntakeShooter intakeShooter,
            ControlStructure structure,
            PoseSelector poseSelector) {

        this.driverXbox = new CommandXboxController(0);;
        this.operatorXbox = new CommandXboxController(1);
        this.driverRightStick = new CommandJoystick(0);
        this.driverLeftStick = new CommandJoystick(1);
        this.swerve = swerve;
        this.elevator = elevator;
        this.arm = arm;
        this.intakeShooter = intakeShooter;
        this.structure = structure;
        this.poseSelector = poseSelector;
    }

    /**
     * Initializes all control bindings and the control chooser.
     * This method should be called once during robot initialization.
     */
    public void init() {
        // Initialize control chooser
        initializeControlChooser();

        // Initialize all standard binding configurations
        singleXboxBindings();
        dualXboxBindings();
        singleStickBindings();
        dualStickBindings();
        singleStickXboxBindings();
        dualStickXboxBindings();
        testBindings();

        // Add custom driver bindings here
        // Example: xAndYBindings();

        // Log initialization
        System.out.println("Robot control bindings initialized successfully");
    }

    /**
     * Initializes the control chooser and adds it to SmartDashboard.
     * Add new driver options here when creating personalized control schemes.
     */
    private void initializeControlChooser() {
        controlChooser.setDefaultOption("Single Xbox", BindingType.SINGLE_XBOX);
        controlChooser.addOption("Dual Xbox", BindingType.DUAL_XBOX);
        controlChooser.addOption("Single Stick", BindingType.SINGLE_STICK);
        controlChooser.addOption("Dual Stick", BindingType.DUAL_STICK);
        controlChooser.addOption("Single Stick and Xbox", BindingType.SINGLE_STICK_XBOX);
        controlChooser.addOption("Dual Stick and Xbox", BindingType.DUAL_STICK_XBOX);
        controlChooser.addOption("Testing", BindingType.TESTING);

        // Add custom driver options here
        // Controls should have standard names. Something "DriverAndOperator" should be fine.
        // Example: controlChooser.addOption("Konnor and Kaden", BindingType.KONNOR_AND_KADEN);

        SmartDashboard.putData("RobotTelemetry/Control Type", controlChooser);
    }

    /**
     * Single Xbox controller bindings for solo operation.
     * All robot controls mapped to one controller with modifier buttons for speed control.
     * <p>
     * Drive Controls:
     * - Left Stick: Translation (forward/back, left/right)
     * - Right Stick X: Rotation
     * - Left Trigger (Hold): Slow mode (30% translation, 20% rotation)
     * - Right Trigger (Hold): Boost mode (100% translation, 75% rotation)
     * <p>
     * Pose Selection:
     * - D-Pad Cardinal Directions: Select reef sides (N, NE, SE, S, SW, NW)
     * - D-Pad Left/Right: Select left/right pose variants
     * - Left/Right Bumpers: Cycle station slots
     * <p>
     * Auto Commands:
     * - A Button: Auto collect from selected station
     * - B Button: Auto score L2
     * - X Button: Auto score L3
     * - Y Button: Auto score L4
     * <p>
     * Settings:
     * - Back: Toggle inverted controls
     * - Right Stick Button: Toggle field/robot relative drive
     */
    private void singleXboxBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.SINGLE_XBOX);
        new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode)
                /* Reef Pose Selection */
                .withReefSelection(driverXbox.povUp(), PoseSelector.ReefSide.NORTH)
                .withReefSelection(driverXbox.povUpRight(), PoseSelector.ReefSide.NORTHEAST)
                .withReefSelection(driverXbox.povDownRight(), PoseSelector.ReefSide.SOUTHEAST)
                .withReefSelection(driverXbox.povDown(), PoseSelector.ReefSide.SOUTH)
                .withReefSelection(driverXbox.povLeft(), PoseSelector.ReefSide.SOUTHWEST)
                .withReefSelection(driverXbox.povUpLeft(), PoseSelector.ReefSide.NORTHWEST)
                /* Left and Right Pose Selection */
                .withLRSelection(driverXbox.povRight(), true)
                .withLRSelection(driverXbox.povLeft(), false)
                /* Slot Pose Cycling */
                .withPoseCycling(driverXbox.rightBumper(), true)
                .withPoseCycling(driverXbox.leftBumper(), true)
                /* Drive Speed Changing */
                .withSlowTranslation(driverXbox.leftTrigger())
                .withBoostTranslation(driverXbox.rightTrigger())
                /* Drive Type Toggles */
                .withHeadingOffset(driverXbox.back())
                .withToggleCentricity(driverXbox.start())
                /* Auto Collect and Score Commands */
                .withAutoCollect(driverXbox.a())
                .withAutoScore(driverXbox.b(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L2)
                .withAutoScore(driverXbox.x(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L3)
                .withAutoScore(driverXbox.y(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L4);
    }

    /**
     * Dual Xbox controller bindings for driver-operator configuration.
     * Split responsibilities between driver and operator for maximum efficiency.
     * <p>
     * Driver Controller:
     * - Left Stick: Translation
     * - Right Stick X: Rotation
     * - Left Trigger (Hold): Slow mode
     * - Right Trigger (Hold): Boost mode
     * - A Button: Auto collect from selected station
     * - B Button: Auto score L2
     * - X Button: Auto score L3
     * - Y Button: Auto score L4
     * - Back: Toggle inverted controls
     * - Right Stick Button: Toggle field/robot relative
     * <p>
     * Operator Controller:
     * - D-Pad: Pose selection (reef sides and left/right)
     * - Left/Right Bumpers: Cycle station slots
     * - Left Stick Y (Hold): Manual elevator control
     * - Right Stick X (Hold): Manual arm control (twist motion)
     * - A Button: Manual collect position
     * - B Button: Manual score L2 position
     * - X Button: Manual score L3 position
     * - Y Button: Manual score L4 position
     * - Left Trigger: Intake
     * - Right Trigger: Shoot (manual override)
     */
    private void dualXboxBindings() {

        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.DUAL_XBOX);

        // Configure driver Xbox controls using helper method
        typicalDriverXboxControls(isMode);
        // Configure operator Xbox controls using helper method
        typicalOperatorXboxControls(isMode);
    }

    /**
     * Single Flight Stick, SINGLE PERSON.
     */
    private void singleStickBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.SINGLE_STICK);

        // Configure dual stick controls using helper method
        typicalDualStickControls(isMode);
    }

    /**
     * Two Flight Sticks, One Person.
     */
    private void dualStickBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.DUAL_STICK);

        // Configure dual stick controls using helper method
        typicalDualStickControls(isMode);
    }

    /**
     * Single Logitech Extreme 3D Pro with Xbox operator configuration.
     * Hybrid control scheme combining joystick precision with Xbox ergonomics.
     * <p>
     * Driver Stick:
     * - X/Y Axes: Translation
     * - Twist: Rotation
     * - Throttle: Speed scaling
     * <p>
     * Operator Xbox:
     * - Same as dual Xbox operator controls
     */
    private void singleStickXboxBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.SINGLE_STICK_XBOX);

        // Configure dual stick controls using helper method
        typicalSingleStickControls(isMode);
    }

    /**
     * Dual Logitech Extreme 3D Pro with Xbox operator configuration.
     * Maximum control flexibility with three input devices.
     * <p>
     * This configuration provides the most comprehensive control scheme,
     * combining dual stick precision for driving with Xbox controller
     * convenience for mechanism operation.
     */
    private void dualStickXboxBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.DUAL_STICK_XBOX);
        // Configure dual stick controls using helper method
        typicalDualStickControls(isMode);

        // Configure operator Xbox controls using helper method
        typicalOperatorXboxControls(isMode);
    }

    /**
     * Test mode bindings for debugging and system identification.
     * Provides direct control over subsystems for testing and calibration.
     * <p>
     * WARNING: Test mode bypasses safety interlocks. Use with caution.
     */
    private void testBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.TESTING);
        new ControlStream(isMode);
    }

    /**
     * Helper method to configure typical driver Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private void typicalDriverXboxControls(Trigger isMode) {
        new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode)
                /* Drive Speed Changing */
                .withSlowTranslation(driverXbox.leftTrigger())
                .withBoostTranslation(driverXbox.rightTrigger())
                /* Drive Type Toggles */
                .withHeadingOffset(driverXbox.back())
                .withToggleCentricity(driverXbox.start())
                /* Auto Collect and Score Commands */
                .withAutoCollect(driverXbox.a())
                .withAutoScore(driverXbox.b(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L2)
                .withAutoScore(driverXbox.x(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L3)
                .withAutoScore(driverXbox.y(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L4);
    }

    /**
     * Helper method to configure typical operator Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private void typicalOperatorXboxControls(Trigger isMode) {
        new ControlStream(isMode)
                /* Reef Pose Selection */
                .withReefSelection(operatorXbox.povUp(), PoseSelector.ReefSide.NORTH)
                .withReefSelection(operatorXbox.povUpRight(), PoseSelector.ReefSide.NORTHEAST)
                .withReefSelection(operatorXbox.povDownRight(), PoseSelector.ReefSide.SOUTHEAST)
                .withReefSelection(operatorXbox.povDown(), PoseSelector.ReefSide.SOUTH)
                .withReefSelection(operatorXbox.povLeft(), PoseSelector.ReefSide.SOUTHWEST)
                .withReefSelection(operatorXbox.povUpLeft(), PoseSelector.ReefSide.NORTHWEST)
                /* Left and Right Pose Selection */
                .withLRSelection(operatorXbox.povRight(), true)
                .withLRSelection(operatorXbox.povLeft(), false)
                /* Slot Pose Cycling */
                .withPoseCycling(operatorXbox.rightBumper(), true)
                .withPoseCycling(operatorXbox.leftBumper(), true)
                /* Manual Mech Control */
                .withElevatorManual(() -> -1 * operatorXbox.getLeftY())
                .withArmManual(() -> -1 * operatorXbox.getRightX())
                .withIntakeShooter(operatorXbox.leftTrigger(), true)
                .withIntakeShooter(operatorXbox.rightTrigger(), false)
                /* Manual Collect and Score Commands */
                .withCollect(operatorXbox.a())
                .withManualScore(operatorXbox.b(), operatorXbox.leftBumper(), ControlStructure.ScoreLevels.SCORE_L2)
                .withManualScore(operatorXbox.x(), operatorXbox.leftBumper(), ControlStructure.ScoreLevels.SCORE_L3)
                .withManualScore(operatorXbox.y(), operatorXbox.leftBumper(), ControlStructure.ScoreLevels.SCORE_L4);
    }

    /**
     * Helper method to configure typical driver single stick controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private void typicalSingleStickControls(Trigger isMode) {
        new ControlStream(
                () -> -1 * driverRightStick.getY(),
                () -> -1 * driverRightStick.getX(),
                () -> -1 * driverRightStick.getTwist(),
                isMode);

        // Single stick bindings <!>
    }

    /**
     * Helper method to configure typical driver dual stick controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private void typicalDualStickControls(Trigger isMode) {
        new ControlStream(
                () -> -1 * driverRightStick.getY(),
                () -> -1 * driverRightStick.getX(),
                () -> -1 * driverLeftStick.getTwist(),
                isMode);

        // Dual stick bindings <!>
    }

    /*
     *
     *  Set Custom controls below this block
     *
     */

    /**
     * Gets the currently selected binding type.
     *
     * @return The active BindingType
     */
    public BindingType getCurrentBindingType() {
        return controlChooser.getSelected();
    }

    /**
     * Gets the control chooser for external access if needed.
     *
     * @return The SendableChooser for control selection
     */
    public SendableChooser<BindingType> getControlChooser() {
        return controlChooser;
    }

    // Control binding type enum
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

    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private class ControlStream {
        /* Trigger used to enable all controls in this Control Stream class */
        protected Optional<Trigger> isMode;
        /* Drive train control constants */
        protected Optional<Double> SLOW_TRANSLATION;
        protected Optional<Double> SLOW_ROTATION;
        protected Optional<Double> NORMAL_TRANSLATION;
        protected Optional<Double> NORMAL_ROTATION;
        protected Optional<Double> BOOST_TRANSLATION;
        protected Optional<Double> BOOST_ROTATION;
        protected Optional<SwerveInputStream> inputStream;
        protected Optional<Command> driveCommand;
        /* Subsystem control constants */
        protected Optional<Double> ELEVATOR_SPEED;
        protected Optional<Double> ARM_SPEED;


        /**
         * Input stream used to streamline the construction of custom input profiles.
         *
         * @param isMode {@link Trigger} used to determine when input should be allowed.
         */
        public ControlStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, Trigger isMode) {
            /* Trigger used to enable all controls in this Control Stream class */
            this.isMode = Optional.of(isMode);
            /* Drive train control constants */
            this.SLOW_TRANSLATION = Optional.of(0.3);
            this.SLOW_ROTATION = Optional.of(0.2);
            this.NORMAL_TRANSLATION = Optional.of(0.8);
            this.NORMAL_ROTATION = Optional.of(0.6);
            this.BOOST_TRANSLATION = Optional.of(1.0);
            this.BOOST_ROTATION = Optional.of(0.75);
            this.inputStream = Optional.of(SwerveInputStream.of(
                            swerve.getSwerveDrive(),
                            x, y)
                    .cubeTranslationControllerAxis(true)
                    .withControllerRotationAxis(rotation)
                    .deadband(Constants.OperatorConstants.DEADBAND)
                    .scaleTranslation(NORMAL_TRANSLATION.get())
                    .scaleRotation(NORMAL_ROTATION.get())
                    .robotRelative(true)
                    .allianceRelativeControl(false)
                    .translationHeadingOffset(Rotation2d.k180deg));
            updateDriveCommand();
            /* Subsystem control constants */
            this.ELEVATOR_SPEED = Optional.of(Elevator.ControlConstants.kElevatorSpeed);
            this.ARM_SPEED = Optional.of(Arm.ControlConstants.kArmSpeed);

            // Set default drive command when enabled
            if (driveCommand.isPresent()) {
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(getDriveCommand())));
            }
        }

        /**
         * Input stream used to streamline the construction of custom input profiles.
         *
         * @param isMode {@link Trigger} used to determine when input should be allowed.
         */
        public ControlStream(Trigger isMode) {
            /* Trigger used to enable all controls in this Control Stream class */
            this.isMode = Optional.of(isMode);
            /* Drive train control constants */
            this.SLOW_TRANSLATION = Optional.of(0.3);
            this.SLOW_ROTATION = Optional.of(0.2);
            this.NORMAL_TRANSLATION = Optional.of(0.8);
            this.NORMAL_ROTATION = Optional.of(0.6);
            this.BOOST_TRANSLATION = Optional.of(1.0);
            this.BOOST_ROTATION = Optional.of(0.75);
            /* Drive Controls not defined, I'll still define default drive variables for adding input streams after init. */
            /* Subsystem control constants */
            this.ELEVATOR_SPEED = Optional.of(Elevator.ControlConstants.kElevatorSpeed);
            this.ARM_SPEED = Optional.of(Arm.ControlConstants.kArmSpeed);
        }

        /*  Operator Type Controls  */

        /**
         * Slows drive train speed.
         *
         * @param shouldSlow button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withSlowTranslation(Trigger shouldSlow) {
            if (isMode.isPresent() && inputStream.isPresent()
                    && SLOW_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
                isMode.get().and(shouldSlow).whileTrue(Commands.runEnd(
                        () -> inputStream.get().scaleTranslation(SLOW_TRANSLATION.get()),
                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
            } else {
                DriverStation.reportWarning("Something not found, Slow Translation failed.", true);
            }
            return this;
        }

        /**
         * Boosts drive train speed.
         *
         * @param shouldBoost button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withBoostTranslation(Trigger shouldBoost) {
            if (isMode.isPresent() && inputStream.isPresent()
                    && BOOST_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
                isMode.get().and(shouldBoost).whileTrue(Commands.runEnd(
                        () -> inputStream.get().scaleTranslation(BOOST_TRANSLATION.get()),
                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
            } else {
                DriverStation.reportWarning("Something not found, Boost Translation failed.", true);
            }
            return this;
        }

        /**
         * Method to switch turning the heading offset on and off.
         * Heading offset can be set with setHeadingOffset(Angle)
         * {Default : 180 degrees}
         *
         * @param shouldOffset button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withHeadingOffset(Trigger shouldOffset) {
            if (isMode.isPresent() && inputStream.isPresent()
                    && BOOST_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
                isMode.get().and(shouldOffset).whileTrue(Commands.runEnd(
                        () -> inputStream.get().scaleTranslation(BOOST_TRANSLATION.get()),
                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
            } else {
                DriverStation.reportWarning("Something not found, Heading offset failed.", true);
            }
            return this;
        }

        /**
         * Method to switch between field and robot centric drives.
         *
         * @param shouldToggleCentricity button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withToggleCentricity(Trigger shouldToggleCentricity) {
            if (isMode.isPresent() && inputStream.isPresent()) {
                isMode.get().and(shouldToggleCentricity).toggleOnTrue(Commands.runEnd(
                        () -> inputStream.get().robotRelative(false).allianceRelativeControl(true),
                        () -> inputStream.get().robotRelative(true).allianceRelativeControl(false)
                ));
            } else {
                DriverStation.reportWarning("Something not found, Toggle Centricity failed.", true);
            }
            return this;
        }

        /**
         * Command to automatically drive to the selected coral station pose and intake until a game piece is sensed.
         *
         * @param shouldAutoCollect button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withAutoCollect(Trigger shouldAutoCollect) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldAutoCollect).whileTrue(structure.autoCollect(driverXbox.rightTrigger()));
            } else {
                DriverStation.reportWarning("isMode not found, Auto Collect failed.", true);
            }
            return this;
        }

        /**
         * Command to automatically drive to the selected reef pose and shoot until the sensor is inactive.
         *
         * @param shouldAutoScore button mapping {@link Trigger} to use.
         * @param shouldBoostSpeed button mapping {@link Trigger} to use for boosting auto drive speed.
         * @param scoreLevel where to score the coral.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withAutoScore(Trigger shouldAutoScore, Trigger shouldBoostSpeed, ControlStructure.ScoreLevels scoreLevel) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldAutoScore).whileTrue(structure.autoScore(scoreLevel, shouldBoostSpeed));
            } else {
                DriverStation.reportWarning("isMode not found, Auto Score failed.", true);
            }
            return this;
        }

        /*  Operator Type Controls  */

        /**
         * Updates reef selection with specified reef side.
         *
         * @param shouldPoseSelection button mapping {@link Trigger} to use.
         * @param reefSide {@link frc.robot.utils.robot.PoseSelector.ReefSide} to select.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withReefSelection(Trigger shouldPoseSelection, PoseSelector.ReefSide reefSide) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldPoseSelection).onTrue(Commands.runOnce(() -> poseSelector.selectReefSide(reefSide)));
            } else {
                DriverStation.reportWarning("isMode not found, Reef Selection failed.", true);
            }
            return this;
        }

        /**
         * Cycles the station slot and cage pose selections.
         *
         * @param shouldPoseCycling button mapping {@link Trigger} to use.
         * @param isUp whether to cycle up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withPoseCycling(Trigger shouldPoseCycling, boolean isUp) {
            if (isMode.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldPoseCycling).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));
                } else {
                    isMode.get().and(shouldPoseCycling).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
                }
            } else {
                DriverStation.reportWarning("isMode not found, Pose Cycling failed.", true);
            }
            return this;
        }

        /**
         * Updates pose selection with specified side.
         *
         * @param shouldPoseSelection button mapping {@link Trigger} to use.
         * @param isRight whether to select right or left pose.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withLRSelection(Trigger shouldPoseSelection, boolean isRight) {
            if (isMode.isPresent()) {
                if (isRight) {
                    isMode.get().and(shouldPoseSelection).onTrue(Commands.runOnce(() -> poseSelector.selectLR(true)));
                } else {
                    isMode.get().and(shouldPoseSelection).onTrue(Commands.runOnce(() -> poseSelector.selectLR(false)));
                }
            } else {
                DriverStation.reportWarning("isMode not found, LR Selection failed.", true);
            }
            return this;
        }

        /**
         * Cycles pose selection with toward side.
         *
         * @param shouldPoseSelection button mapping {@link Trigger} to use.
         * @param isRight whether to cycle right or left.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withLRCycle(Trigger shouldPoseSelection, boolean isRight) {
            if (isMode.isPresent()) {
                if (isRight) {
                    isMode.get().and(shouldPoseSelection).onTrue(Commands.runOnce(poseSelector::cycleReefPoseRight));
                } else {
                    isMode.get().and(shouldPoseSelection).onTrue(Commands.runOnce(poseSelector::cycleReefPoseLeft));
                }
            } else {
                DriverStation.reportWarning("isMode not found, LR Selection failed.", true);
            }
            return this;
        }

        /**
         * Controls the elevator manually.
         * Set speed with setElevatorSpeed(double) or use a withElevatorManual method with defined speed.
         *
         * @param shouldElevatorManual button mapping {@link Trigger} to use.
         * @param isUp whether drive elevator up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withElevatorManual(Trigger shouldElevatorManual, boolean isUp) {
            if (isMode.isPresent() && ELEVATOR_SPEED.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldElevatorManual).whileTrue(elevator.elevCmd(ELEVATOR_SPEED.get()));
                } else {
                    isMode.get().and(shouldElevatorManual).whileTrue(elevator.elevCmd(-ELEVATOR_SPEED.get()));
                }
            } else {
                DriverStation.reportWarning("Something not found, Elevator Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the elevator manually.
         * Set speed with setElevatorSpeed(double) or use a withElevatorManual method with defined speed.
         *
         * @param shouldElevatorManual button mapping {@link Trigger} to use.
         * @param isUp whether drive elevator up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withElevatorManual(Trigger shouldElevatorManual, boolean isUp, double speed) {
            if (isMode.isPresent() && ELEVATOR_SPEED.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldElevatorManual).whileTrue(elevator.elevCmd(speed));
                } else {
                    isMode.get().and(shouldElevatorManual).whileTrue(elevator.elevCmd(-speed));
                }
            } else {
                DriverStation.reportWarning("Something not found, Elevator Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the elevator manually.
         * Set speed with setElevatorSpeed(double) or use a withElevatorManual method with defined speed.
         *
         * @param shouldElevatorManual button mapping {@link Trigger} to use.
         * @param isUp whether drive elevator up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withElevatorManual(Trigger shouldElevatorManual, boolean isUp, Supplier<Double> speed) {
            if (isMode.isPresent() && ELEVATOR_SPEED.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldElevatorManual).whileTrue(elevator.elevCmd(speed));
                } else {
                    isMode.get().and(shouldElevatorManual).whileTrue(elevator.elevCmd(() -> -speed.get()));
                }
            } else {
                DriverStation.reportWarning("Something not found, Elevator Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the elevator manually with an axis. Uses deadband constant as trigger.
         *
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withElevatorManual(Supplier<Double> speed) {
            if (isMode.isPresent()) {
                isMode.get().and(() -> Math.abs(speed.get()) > Constants.OperatorConstants.DEADBAND)
                        .whileTrue(elevator.elevCmd(speed));
            } else {
                DriverStation.reportWarning("Something not found, Elevator Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the arm manually.
         * Set speed with setArmSpeed(double) or use a withArmManual method with defined speed.
         *
         * @param shouldArmManual button mapping {@link Trigger} to use.
         * @param isUp whether drive elevator up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withArmManual(Trigger shouldArmManual, boolean isUp) {
            if (isMode.isPresent() && ARM_SPEED.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldArmManual).whileTrue(arm.armCmd(ARM_SPEED.get()));
                } else {
                    isMode.get().and(shouldArmManual).whileTrue(arm.armCmd(-ARM_SPEED.get()));
                }
            } else {
                DriverStation.reportWarning("Something not found, Arm Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the arm manually.
         * Set speed with setArmSpeed(double) or use a withArmManual method with defined speed.
         *
         * @param shouldArmManual button mapping {@link Trigger} to use.
         * @param isUp whether drive elevator up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withArmManual(Trigger shouldArmManual, boolean isUp, double speed) {
            if (isMode.isPresent() && ARM_SPEED.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldArmManual).whileTrue(arm.armCmd(speed));
                } else {
                    isMode.get().and(shouldArmManual).whileTrue(arm.armCmd(-speed));
                }
            } else {
                DriverStation.reportWarning("Something not found, Arm Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the arm manually.
         * Set speed with setArmSpeed(double) or use a withArmManual method with defined speed.
         *
         * @param shouldArmManual button mapping {@link Trigger} to use.
         * @param isUp whether drive elevator up or down.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withArmManual(Trigger shouldArmManual, boolean isUp, Supplier<Double> speed) {
            if (isMode.isPresent() && ARM_SPEED.isPresent()) {
                if (isUp) {
                    isMode.get().and(shouldArmManual).whileTrue(arm.armCmd(speed));
                } else {
                    isMode.get().and(shouldArmManual).whileTrue(arm.armCmd(() -> -speed.get()));
                }
            } else {
                DriverStation.reportWarning("Something not found, Arm Manual failed.", true);
            }
            return this;
        }

        /**
         * Controls the arm manually with an axis. Uses deadband constant as trigger.
         *
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withArmManual(Supplier<Double> speed) {
            if (isMode.isPresent()) {
                isMode.get().and(() -> Math.abs(speed.get()) > Constants.OperatorConstants.DEADBAND)
                        .whileTrue(arm.armCmd(speed));
            } else {
                DriverStation.reportWarning("Something not found, Arm Manual failed.", true);
            }
            return this;
        }

        /**
         * Runs the intake/shooter motor at predetermined uneditable speeds.
         *
         * @param shouldIntakeShooter button mapping {@link Trigger} to use.
         * @param isIntake whether to intake or shoot
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withIntakeShooter(Trigger shouldIntakeShooter, boolean isIntake) {
            if (isMode.isPresent()) {
                if (isIntake) {
                    isMode.get().and(shouldIntakeShooter).whileTrue(intakeShooter.intake());
                } else {
                    isMode.get().and(shouldIntakeShooter).whileTrue(intakeShooter.shoot());
                }
            } else {
                DriverStation.reportWarning("Something not found, Intake Shooter failed.", true);
            }
            return this;
        }

        /**
         * Command to move structure to coral station pose and intake until a game piece is sensed.
         *
         * @param shouldCollect button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withCollect(Trigger shouldCollect) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldCollect).whileTrue(structure.collect());
            } else {
                DriverStation.reportWarning("isMode not found, Collect failed.", true);
            }
            return this;
        }

        /**
         * Command that when enabled moves structures to selected reef pose then waits for the isReady trigger to then shoot
         * the until the sensor is inactive.
         * Does not drive.
         *
         * @param shouldAutoScore button mapping {@link Trigger} to use.
         * @param isReady when to start scoring.
         * @param scoreLevel where to score the coral.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withManualScore(Trigger shouldAutoScore, Trigger isReady, ControlStructure.ScoreLevels scoreLevel) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldAutoScore).whileTrue(structure.manualScore(scoreLevel, isReady));
            } else {
                DriverStation.reportWarning("isMode not found, Auto Score failed.", true);
            }
            return this;
        }

        /* Control Constant Setting Methods */

        /**
         * Updates the driveCommand from the latest {@link SwerveInputStream}.
         *
         * @return {@link InputStructure} for chaining.
         */
        ControlStream updateDriveCommand() {
            if (inputStream.isPresent()) {
                this.driveCommand = Optional.of(swerve.driveFieldOriented(inputStream.get()));
                swerve.setDefaultCommand(driveCommand.get());
            } else {
                DriverStation.reportWarning("Input stream not found, updateDriveCommand failed.", false);
            }
            return this;
        }

        /**
         * Changes the default drive command.
         * This just sets the commands as the SwerveSubsystem default command.
         *
         * @param driveCommand {@link Command} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setDriveCommand(Command driveCommand) {
            this.driveCommand = Optional.of(driveCommand);
            updateDriveCommand();
            return this;
        }

        /**
         * Method to set the heading offset.
         * Offset is then enabled by using withHeadingOffset(Trigger)
         *
         * @param headingOffset heading offset angle.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setHeadingOffset(Angle headingOffset) {
            if (inputStream.isPresent()) {
                inputStream.get().translationHeadingOffset(new  Rotation2d(headingOffset));
            } else {
                DriverStation.reportWarning("Input Stream not found, setting Heading Offset failed.", true);
            }
            return this;
        }

        /**
         * Changes the {@link ControlStream}'s {@link SwerveInputStream} for controlling the drive train.
         *
         * @param inputStream {@link SwerveInputStream} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setInputStream(SwerveInputStream inputStream) {
            this.inputStream = Optional.of(inputStream);
            return this;
        }

        /**
         * Speed to use to control the elevator when not defining a speed.
         *
         * @param elevatorSpeed Duty cycle speed to use. {-0.0, 1.0}
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setElevatorSpeed(double elevatorSpeed) {
            if (ELEVATOR_SPEED.isPresent()) {
                this.ELEVATOR_SPEED = Optional.of(MathUtil.clamp(elevatorSpeed, -0.0, 1.0));
            } else {
                DriverStation.reportWarning("Elevator Speed is invalid.", true);
            }
            return this;
        }

        /**
         * Speed to use to control the arm when not defining a speed.
         *
         * @param armSpeed Duty cycle speed to use. {-0.0, 1.0}
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setArmSpeed(double armSpeed) {
            if (ARM_SPEED.isPresent()) {
                this.ARM_SPEED = Optional.of(MathUtil.clamp(armSpeed, -0.0, 1.0));
            } else {
                DriverStation.reportWarning("Arn Speed is invalid.", true);
            }
            return this;
        }

        /* Control Constant Getting Methods */

        /**
         * Gets the drive command.
         *
         * @return the {@link SwerveInputStream} or if not found will report a warning and return null.
         */
        Command getDriveCommand() {
            Command command = driveCommand.orElse(null);
            if (command == null) {
                DriverStation.reportWarning("Drive Command is null", false);
                return null;
            } else {
                return command;
            }
        }

        /**
         * Gets the input stream.
         *
         * @return the {@link SwerveInputStream} or if not found will report a warning and return null.
         */
        SwerveInputStream getInputStream() {
            SwerveInputStream stream = inputStream.orElse(null);
            if (stream == null) {
                DriverStation.reportWarning("Input stream is null", false);
                return null;
            } else {
                return stream;
            }
        }

        /* Miscellaneous */

    }
}