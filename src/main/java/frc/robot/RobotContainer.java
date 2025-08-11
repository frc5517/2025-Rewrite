// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.BindingsSelector;
import frc.robot.utils.FunDriveModes;
import frc.robot.utils.PoseSelector;
import frc.robot.utils.maplesim.MapleSim;
import frc.robot.utils.maplesim.opponents.reefscape.kitbot.KitBot;
import frc.robot.utils.maplesim.opponents.reefscape.kitbotpro.KitBotPro;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class RobotContainer {
    private final SendableChooser<BindingsSelector.BindingType> bindingSendable = new SendableChooser<>();
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseSelector poseSelector = new PoseSelector(swerve);
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final AddressableLEDSubsystem led = new AddressableLEDSubsystem();
    private final IntakeShooter intakeShooter = new IntakeShooter(
            swerve, elevator, arm);
    private final ControlStructure structure = new ControlStructure(
            swerve, poseSelector, arm, elevator, intakeShooter, led);
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        bindingsSendableInit();
        configureBindings();
        setupAutonomous();
        KitBotPro[] kitBotPros = new KitBotPro[2];
        KitBot[] kitBots = new KitBot[3];
        kitBots[0] = new KitBot(0, DriverStation.Alliance.Blue);
        kitBots[0].withJoystick(new CommandXboxController(3));
        kitBots[1] = new KitBot(1, DriverStation.Alliance.Blue);
        kitBots[2] = new KitBot(2, DriverStation.Alliance.Blue);
        kitBotPros[0] = new KitBotPro(3, DriverStation.Alliance.Red);
        kitBotPros[0].withJoystick(new CommandXboxController(4));
        kitBotPros[1] = new KitBotPro(4, DriverStation.Alliance.Red);
    }

    private void bindingsSendableInit() {
        bindingSendable.setDefaultOption("Single Xbox", BindingsSelector.BindingType.SINGLE_XBOX);
        bindingSendable.addOption("Dual Xbox", BindingsSelector.BindingType.DUAL_XBOX);
        bindingSendable.addOption("Single Stick", BindingsSelector.BindingType.SINGLE_STICK);
        bindingSendable.addOption("Dual Stick", BindingsSelector.BindingType.DUAL_STICK);
        bindingSendable.addOption("Single Stick and Xbox", BindingsSelector.BindingType.SINGLE_STICK_XBOX);
        bindingSendable.addOption("Dual Stick and Xbox", BindingsSelector.BindingType.DUAL_STICK_XBOX);
        bindingSendable.addOption("Testing", BindingsSelector.BindingType.TESTING);
        SmartDashboard.putData("RobotTelemetry/Control Type", bindingSendable);
    }

    public void configureBindings() {
        singleXboxBindings();
        dualXboxBindings();
        singleStickBindings();
        dualStickBindings();
        singleStickXboxBindings();
        dualStickXboxBindings();
        testBindings();
    }

    public void setupAutonomous() {
        // Named Commands go here
        //NamedCommands.registerCommand("GUI NAME", theCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    //
    // Control binding methods
    //

    private void singleXboxBindings() {
        final SwerveInputStream inputStream = getInputStream(
                () -> driverXbox.getLeftY() * -1,
                () -> driverXbox.getLeftX() * -1,
                () -> driverXbox.getRightX() * -1).copy();
        Command driveXboxCommand = swerve.driveFieldOriented(inputStream);
        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.SINGLE_XBOX);
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveXboxCommand)));
        isMode.and(driverXbox.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isMode.and(driverXbox.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isMode.and(driverXbox.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isMode.and(driverXbox.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isMode.and(driverXbox.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isMode.and(driverXbox.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        // Choose left or right pose, used in station and reef
        isMode.and(driverXbox.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isMode.and(driverXbox.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));

        // Cycle cage and station poses
        isMode.and(driverXbox.leftBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isMode.and(driverXbox.rightBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        // Slow speed while holding the left trigger
        isMode.and(driverXbox.leftTrigger()).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(0.3)
                        .scaleRotation(0.2),
                () -> inputStream.scaleTranslation(0.8)
                        .scaleRotation(0.6)
        ));
        // Boost speed while holding the right trigger
        isMode.and(driverXbox.rightTrigger()).whileTrue(Commands.runEnd(
                () -> inputStream.scaleTranslation(1)
                        .scaleRotation(.75),
                () -> inputStream.scaleTranslation(.8)
                        .scaleRotation(.6)
        ));

        // Toggle to invert controls
        isMode.and(driverXbox.back()).toggleOnTrue(Commands.runEnd(
                () -> inputStream.translationHeadingOffset(true),
                () -> inputStream.translationHeadingOffset(false)
        ));

        // Toggle field or robot relative speeds
        isMode.and(driverXbox.start()).toggleOnTrue(Commands.runEnd(
                () -> inputStream.robotRelative(false)
                        .allianceRelativeControl(true),
                () -> inputStream.robotRelative(true)
                        .allianceRelativeControl(false)
        ));

        // Drive to reef
        isMode.and(driverXbox.a()).whileTrue(structure.autoCollect(driverXbox.rightTrigger()));
        // Drive to station
        isMode.and(driverXbox.b()).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L2, driverXbox.rightTrigger()));
        // Drive to processor
        isMode.and(driverXbox.x()).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L3, driverXbox.rightTrigger()));
        // Drive into climb
        isMode.and(driverXbox.y()).whileTrue(structure.autoScore(ControlStructure.ScoreLevels.SCORE_L4, driverXbox.rightTrigger()));
    }

    private void dualXboxBindings() {
        // TODO Make all bindings
        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.DUAL_XBOX);
        isMode.onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(Commands.none())));
    }

    private void singleStickBindings() {
        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.SINGLE_STICK);
        isMode.onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(Commands.none())));
    }

    private void dualStickBindings() {
        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.DUAL_STICK);
        isMode.onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(Commands.none())));
    }

    private void singleStickXboxBindings() {
        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.SINGLE_STICK_XBOX);
        isMode.onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(Commands.none())));
    }

    private void dualStickXboxBindings() {
        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.DUAL_STICK_XBOX);
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(Commands.none())));
    }

    private void testBindings() {
        final SwerveInputStream inputStream = getInputStream(
                () -> driverXbox.getLeftY() * -1,
                () -> driverXbox.getLeftX() * -1,
                () -> driverXbox.getRightX() * -1).copy();
        Command driveXboxCommand = swerve.driveFieldOriented(inputStream);

        Trigger isMode = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.TESTING);

//        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveXboxCommand)));
        isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(FunDriveModes.carDrive(FunDriveModes.CarType.FWD, swerve, driverXbox, 0.05))));
        isMode.and(driverXbox.start()).onTrue(Commands.runOnce(() -> MapleSim.addCoralAllStations(false)));
        isMode.and(driverXbox.back()).onTrue(Commands.runOnce(MapleSim::clearMatchData));

        isMode.and(driverXbox.a()).whileTrue(elevator.elevCmd(.5).alongWith(arm.armCmd(Arm.ControlConstants.kArmSpeed)));
        isMode.and(driverXbox.b()).whileTrue(elevator.elevCmd(-.5).alongWith(arm.armCmd(-Arm.ControlConstants.kArmSpeed)));
        isMode.and(driverXbox.x()).whileTrue(arm.setAngle(Degrees.of(0)));
        isMode.and(driverXbox.y()).whileTrue(arm.setAngle(Degrees.of(35)));
        isMode.and(driverXbox.start()).whileTrue(arm.sysId().alongWith(elevator.sysId()));

        isMode.and(driverXbox.leftBumper()).whileTrue(intakeShooter.intake());
        isMode.and(driverXbox.rightBumper()).whileTrue(intakeShooter.shoot());
    }

    public SwerveInputStream getInputStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        return SwerveInputStream.of(
                        swerve.getSwerveDrive(),
                        x, y)
                .cubeTranslationControllerAxis(true)
                .withControllerRotationAxis(rotation)
                .deadband(Constants.OperatorConstants.DEADBAND)
                .scaleTranslation(.8)
                .scaleRotation(.4)
                .robotRelative(true)
                .allianceRelativeControl(false)
                .translationHeadingOffset(Rotation2d.k180deg);
    }
}
