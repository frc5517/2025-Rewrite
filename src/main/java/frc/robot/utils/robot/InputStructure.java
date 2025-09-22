package frc.robot.utils.robot;

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
import frc.robot.utils.FunDriveModes;
import frc.robot.utils.maplesim.MapleSim;
import swervelib.SwerveInputStream;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

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
 * <p>
 * To add custom driver bindings:
 * 1. Create a new method following the naming pattern: [driverName]Bindings()
 * 2. Add the driver option to BindingType enum
 * 3. Add option to initializeControlChooser()
 * 4. Call the method from init()
 *
 * @author Team [Your Team Number]
 * @version 2025 Season
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
        // Local speed constants for this control scheme
        final double SLOW_TRANSLATION = 0.3;
        final double SLOW_ROTATION = 0.2;
        final double NORMAL_TRANSLATION = 0.8;
        final double NORMAL_ROTATION = 0.6;
        final double BOOST_TRANSLATION = 1.0;
        final double BOOST_ROTATION = 0.75;

        // Create input stream for drive control
        final SwerveInputStream inputStream = getInputStream(
                () -> driverXbox.getLeftY() * -1,
                () -> driverXbox.getLeftX() * -1,
                () -> driverXbox.getRightX() * -1,
                NORMAL_TRANSLATION,
                NORMAL_ROTATION).copy();

        // Create drive command
        Command driveXboxCommand = swerve.driveFieldOriented(inputStream);

        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.SINGLE_XBOX);

        // Set default drive command when enabled
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveXboxCommand)));

        // Reef pose selection using D-Pad
        isMode.and(driverXbox.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isMode.and(driverXbox.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isMode.and(driverXbox.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isMode.and(driverXbox.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isMode.and(driverXbox.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isMode.and(driverXbox.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        // Choose left or right pose variant
        isMode.and(driverXbox.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isMode.and(driverXbox.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));

        // Cycle through station slots
        isMode.and(driverXbox.leftBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isMode.and(driverXbox.rightBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        // Slow speed mode - hold left trigger
        isMode.and(driverXbox.leftTrigger()).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(SLOW_TRANSLATION)
                        .scaleRotation(SLOW_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION)
                        .scaleRotation(NORMAL_ROTATION)
        ));

        // Boost speed mode - hold right trigger
        isMode.and(driverXbox.rightTrigger()).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(BOOST_TRANSLATION)
                        .scaleRotation(BOOST_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION)
                        .scaleRotation(NORMAL_ROTATION)
        ));

        // Toggle inverted controls
        isMode.and(driverXbox.back()).toggleOnTrue(Commands.runEnd(
                () -> inputStream.translationHeadingOffset(true),
                () -> inputStream.translationHeadingOffset(false)
        ));

        // Toggle field-relative or robot-relative drive
        isMode.and(driverXbox.rightStick()).toggleOnTrue(Commands.runEnd(
                () -> inputStream.robotRelative(false)
                        .allianceRelativeControl(true),
                () -> inputStream.robotRelative(true)
                        .allianceRelativeControl(false)
        ));

        // Auto commands for scoring and collecting
        isMode.and(driverXbox.a()).whileTrue(structure.autoCollect(driverXbox.rightTrigger()));
        isMode.and(driverXbox.b()).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L2, driverXbox.rightTrigger()));
        isMode.and(driverXbox.x()).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L3, driverXbox.rightTrigger()));
        isMode.and(driverXbox.y()).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L4, driverXbox.rightTrigger()));
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

        // Calls for the regular xbox controls.
        typicalDriverXboxControls(isMode);
        // Operator pose selection
        isMode.and(operatorXbox.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isMode.and(operatorXbox.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isMode.and(operatorXbox.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isMode.and(operatorXbox.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isMode.and(operatorXbox.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isMode.and(operatorXbox.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));
        isMode.and(operatorXbox.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isMode.and(operatorXbox.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));

        // Operator station slot cycling
        isMode.and(operatorXbox.leftBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isMode.and(operatorXbox.rightBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        // Operator manual elevator control with deadband
        Trigger elevatorManual = new Trigger(() -> Math.abs(operatorXbox.getLeftY()) > 0.1);
        isMode.and(elevatorManual).whileTrue(
                elevator.elevCmd(operatorXbox.getLeftY() * -Elevator.ControlConstants.kElevatorSpeed)
        );

        // Operator manual arm control with deadband
        Trigger armManual = new Trigger(() -> Math.abs(operatorXbox.getRightX()) > 0.1);
        isMode.and(armManual).whileTrue(
                arm.armCmd(operatorXbox.getRightX() * Arm.ControlConstants.kArmSpeed)
        );

        // Operator position presets
        isMode.and(operatorXbox.a()).whileTrue(structure.collect());

        // Manual score positions with confirmation trigger
        isMode.and(operatorXbox.b()).whileTrue(structure.manualScore(ControlStructure.ScoreLevels.SCORE_L2, operatorXbox.leftBumper()));
        isMode.and(operatorXbox.x()).whileTrue(structure.manualScore(ControlStructure.ScoreLevels.SCORE_L3, operatorXbox.leftBumper()));
        isMode.and(operatorXbox.y()).whileTrue(structure.manualScore(ControlStructure.ScoreLevels.SCORE_L4, operatorXbox.leftBumper()));

        // Operator intake/shoot controls
        isMode.and(operatorXbox.leftTrigger()).whileTrue(intakeShooter.intake());
        isMode.and(operatorXbox.rightTrigger()).whileTrue(intakeShooter.shoot());
    }

    /**
     * Single Logitech Extreme 3D Pro joystick bindings for solo operation.
     * All controls mapped to one joystick with throttle for speed control.
     * <p>
     * Stick Controls:
     * - X/Y Axes: Translation
     * - Twist (Z-Axis): Rotation
     * - Throttle: Speed scaling (forward = slow, back = fast)
     * <p>
     * Buttons:
     * - Button 1 (Trigger): Auto collect
     * - Button 2 (Thumb): Auto score L2
     * - Button 3: Auto score L3
     * - Button 4: Auto score L4
     * - Button 5: Toggle field/robot relative
     * - Button 6: Toggle inverted controls
     * - Button 7: Intake
     * - Button 8: Shoot
     * - Button 9: Manual elevator up
     * - Button 10: Manual elevator down
     * - Button 11: Cycle station slot down
     * - Button 12: Cycle station slot up
     * - POV Hat: Direct pose selection (8-way)
     */
    private void singleStickBindings() {
        // Local speed range constants
        final double MIN_TRANSLATION = 0.3;
        final double MAX_TRANSLATION = 1.0;
        final double MIN_ROTATION = 0.2;
        final double MAX_ROTATION = 0.75;

        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.SINGLE_STICK);

        final SwerveInputStream inputStream = getInputStream(
                () -> driverRightStick.getY() * -1,
                () -> driverRightStick.getX() * -1,
                () -> driverRightStick.getTwist() * -1,
                0.65, // Start with moderate speed
                0.4).copy();
        Command driveStickCommand = swerve.driveFieldOriented(inputStream);
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveStickCommand)));

        // Dynamic speed scaling with throttle lever
        isMode.whileTrue(Commands.run(() -> {
            double throttle = (1.0 - driverRightStick.getThrottle()) / 2.0; // Normalize -1 to 1 range to 0 to 1
            inputStream.scaleTranslation(MIN_TRANSLATION + (throttle * (MAX_TRANSLATION - MIN_TRANSLATION)));
            inputStream.scaleRotation(MIN_ROTATION + (throttle * (MAX_ROTATION - MIN_ROTATION)));
        }));

        // Auto commands on primary buttons
        isMode.and(driverRightStick.button(1)).whileTrue(structure.autoCollect(driverRightStick.button(2)));
        isMode.and(driverRightStick.button(2)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L2, driverRightStick.button(1)));
        isMode.and(driverRightStick.button(3)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L3, driverRightStick.button(1)));
        isMode.and(driverRightStick.button(4)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L4, driverRightStick.button(1)));

        // Drive mode settings
        isMode.and(driverRightStick.button(5)).toggleOnTrue(Commands.runEnd(
                () -> inputStream.robotRelative(false).allianceRelativeControl(true),
                () -> inputStream.robotRelative(true).allianceRelativeControl(false)
        ));
        isMode.and(driverRightStick.button(6)).toggleOnTrue(Commands.runEnd(
                () -> inputStream.translationHeadingOffset(true),
                () -> inputStream.translationHeadingOffset(false)
        ));

        // Manual mechanism controls
        isMode.and(driverRightStick.button(7)).whileTrue(intakeShooter.intake());
        isMode.and(driverRightStick.button(8)).whileTrue(intakeShooter.shoot());
        isMode.and(driverRightStick.button(9)).whileTrue(elevator.elevCmd(0.5));
        isMode.and(driverRightStick.button(10)).whileTrue(elevator.elevCmd(-0.5));

        // Pose selection with POV hat (8-way)
        isMode.and(driverRightStick.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isMode.and(driverRightStick.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isMode.and(driverRightStick.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isMode.and(driverRightStick.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isMode.and(driverRightStick.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isMode.and(driverRightStick.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isMode.and(driverRightStick.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));
        isMode.and(driverRightStick.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        // Station slot cycling
        isMode.and(driverRightStick.button(11)).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isMode.and(driverRightStick.button(12)).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));
    }

    /**
     * Dual Logitech Extreme 3D Pro joystick bindings for advanced driver control.
     * Split controls across two joysticks for maximum precision.
     * <p>
     * Left Stick:
     * - X/Y Axes: Primary translation
     * - Throttle: Translation speed scaling
     * - Button 1: Slow mode override
     * - Button 2: Boost mode override
     * - Button 7: Intake
     * - Button 8: Shoot
     * - POV Hat: Pose selection
     * - Button 11/12: Station cycling
     * <p>
     * Right Stick:
     * - X Axis: Fine translation adjustment (30% scale)
     * - Twist: Rotation control
     * - Throttle: Rotation speed scaling
     * - Button 1 (Trigger): Auto collect
     * - Button 2: Auto score L2
     * - Button 3: Auto score L3
     * - Button 4: Auto score L4
     * <p>
     * Common:
     * - Either Button 5: Toggle field/robot relative
     * - Either Button 6: Toggle inverted controls
     */
    private void dualStickBindings() {
        // Local speed constants
        final double SLOW_TRANSLATION = 0.3;
        final double SLOW_ROTATION = 0.2;
        final double NORMAL_TRANSLATION = 0.8;
        final double NORMAL_ROTATION = 0.6;
        final double BOOST_TRANSLATION = 1.0;
        final double BOOST_ROTATION = 0.75;
        final double FINE_ADJUSTMENT_SCALE = 0.3;

        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.DUAL_STICK);

        // Combine inputs from both sticks for precise control
        final SwerveInputStream inputStream = getInputStream(
                () -> driverRightStick.getY() * -1,
                () -> driverRightStick.getX() * -1 + driverLeftStick.getX() * -FINE_ADJUSTMENT_SCALE,
                () -> driverLeftStick.getTwist() * -1,
                NORMAL_TRANSLATION,
                NORMAL_ROTATION).copy();
        Command driveStickCommand = swerve.driveFieldOriented(inputStream);
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveStickCommand)));

        // Independent throttle controls for translation and rotation
        isMode.whileTrue(Commands.run(() -> {
            double leftThrottle = (1.0 - driverRightStick.getThrottle()) / 2.0;
            double rightThrottle = (1.0 - driverLeftStick.getThrottle()) / 2.0;
            inputStream.scaleTranslation(0.3 + (leftThrottle * 0.7));
            inputStream.scaleRotation(0.2 + (rightThrottle * 0.55));
        }));

        // Speed modifiers on left stick
        isMode.and(driverRightStick.button(1)).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(SLOW_TRANSLATION).scaleRotation(SLOW_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION).scaleRotation(NORMAL_ROTATION)
        ));
        isMode.and(driverRightStick.button(2)).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(BOOST_TRANSLATION).scaleRotation(BOOST_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION).scaleRotation(NORMAL_ROTATION)
        ));

        // Manual controls on left stick
        isMode.and(driverRightStick.button(7)).whileTrue(intakeShooter.intake());
        isMode.and(driverRightStick.button(8)).whileTrue(intakeShooter.shoot());

        // Auto commands on right stick
        isMode.and(driverLeftStick.button(1)).whileTrue(structure.autoCollect(driverLeftStick.button(2)));
        isMode.and(driverLeftStick.button(2)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L2, driverLeftStick.button(1)));
        isMode.and(driverLeftStick.button(3)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L3, driverLeftStick.button(1)));
        isMode.and(driverLeftStick.button(4)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L4, driverLeftStick.button(1)));

        // Settings available on either stick
        isMode.and(driverRightStick.button(5).or(driverLeftStick.button(5))).toggleOnTrue(Commands.runEnd(
                () -> inputStream.robotRelative(false).allianceRelativeControl(true),
                () -> inputStream.robotRelative(true).allianceRelativeControl(false)
        ));
        isMode.and(driverRightStick.button(6).or(driverLeftStick.button(6))).toggleOnTrue(Commands.runEnd(
                () -> inputStream.translationHeadingOffset(true),
                () -> inputStream.translationHeadingOffset(false)
        ));

        // Pose selection on left stick POV
        isMode.and(driverRightStick.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isMode.and(driverRightStick.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isMode.and(driverRightStick.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isMode.and(driverRightStick.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isMode.and(driverRightStick.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isMode.and(driverRightStick.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isMode.and(driverRightStick.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));
        isMode.and(driverRightStick.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        // Station slots on left stick
        isMode.and(driverRightStick.button(11)).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isMode.and(driverRightStick.button(12)).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));
    }

    /**
     * Single Logitech Extreme 3D Pro with Xbox operator configuration.
     * Hybrid control scheme combining joystick precision with Xbox ergonomics.
     * <p>
     * Driver Stick:
     * - X/Y Axes: Translation
     * - Twist: Rotation
     * - Throttle: Speed scaling
     * - Button 1 (Trigger): Boost mode
     * - Button 2: Slow mode
     * - Button 3: Auto collect
     * - Button 4: Auto score L2
     * - Button 5: Auto score L3
     * - Button 6: Auto score L4
     * - Button 7: Toggle field/robot relative
     * - Button 8: Toggle inverted controls
     * <p>
     * Operator Xbox:
     * - Same as dual Xbox operator controls
     */
    private void singleStickXboxBindings() {
        // Local speed constants
        final double SLOW_TRANSLATION = 0.3;
        final double SLOW_ROTATION = 0.2;
        final double NORMAL_TRANSLATION = 0.8;
        final double NORMAL_ROTATION = 0.6;
        final double BOOST_TRANSLATION = 1.0;
        final double BOOST_ROTATION = 0.75;

        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.SINGLE_STICK_XBOX);

        // Driver joystick controls
        final SwerveInputStream inputStream = getInputStream(
                () -> driverRightStick.getY() * -1,
                () -> driverRightStick.getX() * -1,
                () -> driverRightStick.getTwist() * -1,
                NORMAL_TRANSLATION,
                NORMAL_ROTATION).copy();
        Command driveStickCommand = swerve.driveFieldOriented(inputStream);
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveStickCommand)));

        // Throttle-based speed scaling
        isMode.whileTrue(Commands.run(() -> {
            double throttle = (1.0 - driverRightStick.getThrottle()) / 2.0;
            inputStream.scaleTranslation(0.3 + (throttle * 0.7));
            inputStream.scaleRotation(0.2 + (throttle * 0.55));
        }));

        // Driver speed overrides
        isMode.and(driverRightStick.button(1)).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(BOOST_TRANSLATION).scaleRotation(BOOST_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION).scaleRotation(NORMAL_ROTATION)
        ));
        isMode.and(driverRightStick.button(2)).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(SLOW_TRANSLATION).scaleRotation(SLOW_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION).scaleRotation(NORMAL_ROTATION)
        ));

        // Driver auto commands
        isMode.and(driverRightStick.button(3)).whileTrue(structure.autoCollect(driverRightStick.button(1)));
        isMode.and(driverRightStick.button(4)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L2, driverRightStick.button(1)));
        isMode.and(driverRightStick.button(5)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L3, driverRightStick.button(1)));
        isMode.and(driverRightStick.button(6)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L4, driverRightStick.button(1)));

        // Driver settings
        isMode.and(driverRightStick.button(7)).toggleOnTrue(Commands.runEnd(
                () -> inputStream.robotRelative(false).allianceRelativeControl(true),
                () -> inputStream.robotRelative(true).allianceRelativeControl(false)
        ));
        isMode.and(driverRightStick.button(8)).toggleOnTrue(Commands.runEnd(
                () -> inputStream.translationHeadingOffset(true),
                () -> inputStream.translationHeadingOffset(false)
        ));

        // Configure operator Xbox controls using helper method
        typicalOperatorXboxControls(isMode);
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
        // Local speed constants
        final double SLOW_TRANSLATION = 0.3;
        final double SLOW_ROTATION = 0.2;
        final double NORMAL_TRANSLATION = 0.8;
        final double NORMAL_ROTATION = 0.6;
        final double BOOST_TRANSLATION = 1.0;
        final double BOOST_ROTATION = 0.75;
        final double FINE_ADJUSTMENT_SCALE = 0.3;

        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.DUAL_STICK_XBOX);

        // Dual stick drive controls
        final SwerveInputStream inputStream = getInputStream(
                () -> driverRightStick.getY() * -1,
                () -> driverRightStick.getX() * -1 + driverLeftStick.getX() * -FINE_ADJUSTMENT_SCALE,
                () -> driverLeftStick.getTwist() * -1,
                NORMAL_TRANSLATION,
                NORMAL_ROTATION).copy();
        Command driveStickCommand = swerve.driveFieldOriented(inputStream);
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveStickCommand)));

        // Independent throttle controls
        isMode.whileTrue(Commands.run(() -> {
            double leftThrottle = (1.0 - driverRightStick.getThrottle()) / 2.0;
            double rightThrottle = (1.0 - driverLeftStick.getThrottle()) / 2.0;
            inputStream.scaleTranslation(0.3 + (leftThrottle * 0.7));
            inputStream.scaleRotation(0.2 + (rightThrottle * 0.55));
        }));

        // Speed modifiers on left stick
        isMode.and(driverRightStick.button(1)).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(SLOW_TRANSLATION).scaleRotation(SLOW_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION).scaleRotation(NORMAL_ROTATION)
        ));
        isMode.and(driverRightStick.button(2)).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(BOOST_TRANSLATION).scaleRotation(BOOST_ROTATION),
                () -> inputStream.scaleTranslation(NORMAL_TRANSLATION).scaleRotation(NORMAL_ROTATION)
        ));

        // Auto commands on right stick
        isMode.and(driverLeftStick.button(1)).whileTrue(structure.autoCollect(driverLeftStick.button(2)));
        isMode.and(driverLeftStick.button(2)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L2, driverLeftStick.button(1)));
        isMode.and(driverLeftStick.button(3)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L3, driverLeftStick.button(1)));
        isMode.and(driverLeftStick.button(4)).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L4, driverLeftStick.button(1)));

        // Settings on either stick
        isMode.and(driverRightStick.button(5).or(driverLeftStick.button(5))).toggleOnTrue(Commands.runEnd(
                () -> inputStream.robotRelative(false).allianceRelativeControl(true),
                () -> inputStream.robotRelative(true).allianceRelativeControl(false)
        ));
        isMode.and(driverRightStick.button(6).or(driverLeftStick.button(6))).toggleOnTrue(Commands.runEnd(
                () -> inputStream.translationHeadingOffset(true),
                () -> inputStream.translationHeadingOffset(false)
        ));

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
        final SwerveInputStream inputStream = getInputStream(
                () -> driverXbox.getLeftY() * -1,
                () -> driverXbox.getLeftX() * -1,
                () -> driverXbox.getRightX() * -1,
                0.8,
                0.6).copy();

        Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.TESTING);

        // Enable fun drive mode for testing (car simulation)
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() ->
                swerve.setDefaultCommand(FunDriveModes.carDrive(FunDriveModes.CarType.FWD, swerve, driverXbox, 0.05))));

        // Simulation controls
        isMode.and(driverXbox.start()).onTrue(Commands.runOnce(() -> MapleSim.addCoralAllStations(false)));
        isMode.and(driverXbox.back()).onTrue(Commands.runOnce(MapleSim::clearMatchData));

        // Direct mechanism testing with proper constants
        isMode.and(driverXbox.a()).whileTrue(
                elevator.elevCmd(Elevator.ControlConstants.kElevatorSpeed)
                        .alongWith(arm.armCmd(Arm.ControlConstants.kArmSpeed))
        );
        isMode.and(driverXbox.b()).whileTrue(
                elevator.elevCmd(-Elevator.ControlConstants.kElevatorSpeed)
                        .alongWith(arm.armCmd(-Arm.ControlConstants.kArmSpeed))
        );

        // Arm position testing
        isMode.and(driverXbox.x()).whileTrue(arm.setAngle(Degrees.of(0)));
        isMode.and(driverXbox.y()).whileTrue(arm.setAngle(Degrees.of(35)));

        // System identification
        isMode.and(driverXbox.start()).whileTrue(arm.sysId().alongWith(elevator.sysId()));

        // Intake/shooter testing
        isMode.and(driverXbox.leftBumper()).whileTrue(intakeShooter.intake());
        isMode.and(driverXbox.rightBumper()).whileTrue(intakeShooter.shoot());
    }

    /**
     * Helper method to configure typical driver Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private void typicalDriverXboxControls(Trigger isMode) {
        // Local speed constants
        ControlStream driverStream = new ControlStream(isMode)
                .withSlowTranslation(driverXbox.leftTrigger())
                .withBoostTranslation(driverXbox.rightTrigger())
                .withHeadingOffset(driverXbox.back())
                .withToggleCentricity(driverXbox.start())
                .withAutoCollect(driverXbox.a())
                .withAutoScore(driverXbox.b(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L2)
                .withAutoScore(driverXbox.x(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L3)
                .withAutoScore(driverXbox.y(), driverXbox.rightTrigger(), ControlStructure.ScoreLevels.SCORE_L4);
        // if stream isn't found it should return null to make it error when it tries to make a null command.
        Command driveXboxCommand = swerve.driveFieldOriented(driverStream.inputStream.orElse(null));
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveXboxCommand)));
    }

    /**
     * Helper method to configure typical operator Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private void typicalOperatorXboxControls(Trigger isMode) {
        // Pose selection
        isMode.and(operatorXbox.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isMode.and(operatorXbox.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isMode.and(operatorXbox.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isMode.and(operatorXbox.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isMode.and(operatorXbox.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isMode.and(operatorXbox.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));
        isMode.and(operatorXbox.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isMode.and(operatorXbox.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));

        // Station cycling
        isMode.and(operatorXbox.leftBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isMode.and(operatorXbox.rightBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        // Manual elevator control with deadband
        Trigger elevatorManual = new Trigger(() -> Math.abs(operatorXbox.getLeftY()) > 0.1);
        isMode.and(elevatorManual).whileTrue(
                Commands.run(() -> {
                    elevator.getElevator().set(operatorXbox.getLeftY() * -Elevator.ControlConstants.kElevatorSpeed);
                }, elevator)
        );

        // Manual arm control with deadband
        Trigger armManual = new Trigger(() -> Math.abs(operatorXbox.getRightX()) > 0.1);
        isMode.and(armManual).whileTrue(
                Commands.run(() -> {
                    arm.getArm().set(operatorXbox.getRightX() * Arm.ControlConstants.kArmSpeed);
                }, arm)
        );

        // Position presets
        isMode.and(operatorXbox.a()).whileTrue(
                elevator.setHeight(Elevator.ControlConstants.kStationSetpoint)
                        .alongWith(arm.setAngle(Arm.ControlConstants.kStationSetpoint))
        );

        // Manual score positions
        isMode.and(operatorXbox.b()).whileTrue(structure.manualScore(ControlStructure.ScoreLevels.SCORE_L2, operatorXbox.leftBumper()));
        isMode.and(operatorXbox.x()).whileTrue(structure.manualScore(ControlStructure.ScoreLevels.SCORE_L3, operatorXbox.leftBumper()));
        isMode.and(operatorXbox.y()).whileTrue(structure.manualScore(ControlStructure.ScoreLevels.SCORE_L4, operatorXbox.leftBumper()));

        // Intake/shoot
        isMode.and(operatorXbox.leftTrigger()).whileTrue(intakeShooter.intake());
        isMode.and(operatorXbox.rightTrigger()).whileTrue(intakeShooter.shoot());
    }

    /*
     *
     *  Set Custom controls below this block
     *
     */

    /**
     * Creates a configured SwerveInputStream for drive control.
     * Applies standard deadband, scaling, and control mode settings.
     *
     * @param x                Forward/backward axis supplier
     * @param y                Left/right axis supplier
     * @param rotation         Rotation axis supplier
     * @param translationScale Default translation scaling
     * @param rotationScale    Default rotation scaling
     * @return Configured SwerveInputStream ready for use
     */
    private SwerveInputStream getInputStream(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotation,
            double translationScale,
            double rotationScale) {
        return SwerveInputStream.of(
                        swerve.getSwerveDrive(),
                        x, y)
                .cubeTranslationControllerAxis(true)
                .withControllerRotationAxis(rotation)
                .deadband(Constants.OperatorConstants.DEADBAND)
                .scaleTranslation(translationScale)
                .scaleRotation(rotationScale)
                .robotRelative(true)
                .allianceRelativeControl(false)
                .translationHeadingOffset(Rotation2d.k180deg);
    }

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

    /**
     * Example template for creating personalized driver bindings.
     * Copy this method and customize for individual drivers.
     * <p>
     * Steps to add a new driver:
     * 1. Copy this method and rename it (e.g., johnDriverBindings())
     * 2. Add a new enum value to BindingType (e.g., DRIVER_JOHN)
     * 3. Add the option to initializeControlChooser()
     * 4. Call the new method from init()
     * 5. Customize speeds and button mappings as desired
     */
    private void exampleCustomDriverBindings() {
        // Custom speed constants for this driver
        final double SLOW_TRANSLATION = 0.25;  // Even slower for precision
        final double SLOW_ROTATION = 0.15;
        final double NORMAL_TRANSLATION = 0.7;  // Slightly slower normal speed
        final double NORMAL_ROTATION = 0.5;
        final double BOOST_TRANSLATION = 0.9;   // Slightly limited boost
        final double BOOST_ROTATION = 0.7;

        // Create custom binding trigger
        // Trigger isMode = new Trigger(() -> controlChooser.getSelected() == BindingType.DRIVER_EXAMPLE);

        // Configure controls as desired...
    }

    // Control binding type enumeration
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

    private class ControlStream {
        // Local speed constants
        protected Optional<Double> SLOW_TRANSLATION;
        protected Optional<Double> SLOW_ROTATION;
        protected Optional<Double> NORMAL_TRANSLATION;
        protected Optional<Double> NORMAL_ROTATION;
        protected Optional<Double> BOOST_TRANSLATION;
        protected Optional<Double> BOOST_ROTATION;
        protected Optional<SwerveInputStream> inputStream;
        protected Optional<Trigger> isMode;

        public ControlStream(Trigger isMode) {
            this.SLOW_TRANSLATION = Optional.of(0.3);
            this.SLOW_ROTATION = Optional.of(0.2);
            this.NORMAL_TRANSLATION = Optional.of(0.8);
            this.NORMAL_ROTATION = Optional.of(0.6);
            this.BOOST_TRANSLATION = Optional.of(1.0);
            this.BOOST_ROTATION = Optional.of(0.75);

            this.inputStream = Optional.of(InputStructure.this.getInputStream(
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1,
                    () -> driverXbox.getRightX() * -1,
                    NORMAL_TRANSLATION.get(),
                    NORMAL_ROTATION.get()).copy());

            this.isMode = Optional.of(isMode);
        }

        /**
         *
         * @param inputStream
         * @return
         */
        ControlStream withDifferentInputStream(SwerveInputStream inputStream) {
            this.inputStream = Optional.of(inputStream);
            return this;
        }

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

        ControlStream setHeadingOffset(Angle headingOffset) {
            if (inputStream.isPresent()) {
                inputStream.get().translationHeadingOffset(new  Rotation2d(headingOffset));
            } else {
                DriverStation.reportWarning("Input Stream not found, setting Heading Offset failed.", true);
            }
            return this;
        }

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

        ControlStream withAutoCollect(Trigger shouldAutoCollect) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldAutoCollect).whileTrue(structure.autoCollect(driverXbox.rightTrigger()));
            } else {
                DriverStation.reportWarning("isMode not found, Auto Collect failed.", true);
            }
            return this;
        }

        ControlStream withAutoScore(Trigger shouldAutoScore, Trigger shouldBoostSpeed, ControlStructure.ScoreLevels scoreLevel) {
            if (isMode.isPresent()) {
                isMode.get().and(shouldAutoScore).whileTrue(structure.autoScore(scoreLevel, shouldBoostSpeed));
            } else {
                DriverStation.reportWarning("isMode not found, Auto Score failed.", true);
            }
            return this;
        }

        /**
         * Gets the input stream.
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
    }
}