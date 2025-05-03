// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.BindingsSelector;
import frc.robot.utils.PoseSelector;
import swervelib.SwerveInputStream;

import java.io.File;


public class RobotContainer
{
    private final SendableChooser<BindingsSelector.BindingType> bindingSendable  = new SendableChooser<>();
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseSelector poseSelector = new PoseSelector(swerveSubsystem);

    public RobotContainer()
    {
        bindingsSendableInit();
        configureBindings();
    }

    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }

    private void bindingsSendableInit()
    {
        bindingSendable.setDefaultOption("Single Xbox", BindingsSelector.BindingType.SINGLE_XBOX);
        bindingSendable.addOption("Dual Xbox", BindingsSelector.BindingType.DUAL_XBOX);
        bindingSendable.addOption("Single Stick", BindingsSelector.BindingType.SINGLE_STICK);
        bindingSendable.addOption("Dual Stick", BindingsSelector.BindingType.DUAL_STICK);
        bindingSendable.addOption("Single Stick and Xbox", BindingsSelector.BindingType.SINGLE_STICK_XBOX);
        bindingSendable.addOption("Dual Stick and Xbox", BindingsSelector.BindingType.DUAL_STICK_XBOX);
        SmartDashboard.putData("Control Type", bindingSendable);
    }
    
    public void configureBindings()
    {
        // Always on controls
        // TO-DO: add always on to force update controls.

        switch(bindingSendable.getSelected()) {
            case SINGLE_XBOX:
                SwerveInputStream swerveInputStream = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                () -> driverXbox.getLeftY() * -1,
                                () -> driverXbox.getLeftX() * -1)
                        .cubeTranslationControllerAxis(true)
                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                        .deadband(Constants.OperatorConstants.DEADBAND)
                        .scaleTranslation(.8)
                        .scaleRotation(.4)
                        .robotRelative(true)
                        .allianceRelativeControl(false)
                        .translationHeadingOffset(Rotation2d.k180deg);
                Command driveCommand = swerveSubsystem.driveFieldOriented(swerveInputStream);
                swerveSubsystem.setDefaultCommand(driveCommand);

                driverXbox.povUp().onTrue(Commands.runOnce(poseSelector::selectNorth));
                driverXbox.povUpRight().onTrue(Commands.runOnce(poseSelector::selectNorthEast));
                driverXbox.povDownRight().onTrue(Commands.runOnce(poseSelector::selectSouthEast));
                driverXbox.povDown().onTrue(Commands.runOnce(poseSelector::selectSouth));
                driverXbox.povDownLeft().onTrue(Commands.runOnce(poseSelector::selectSouthWest));
                driverXbox.povUpLeft().onTrue(Commands.runOnce(poseSelector::selectNorthWest));

                // Choose left or right pose, used in station and reef
                driverXbox.povRight().onTrue(Commands.runOnce(poseSelector::selectRight));
                driverXbox.povLeft().onTrue(Commands.runOnce(poseSelector::selectLeft));

                // Cycle cage and station poses
                driverXbox.leftBumper().onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
                driverXbox.rightBumper().onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

                // Slow speed while holding left trigger
                driverXbox.leftTrigger().whileTrue(Commands.runEnd(
                        () -> swerveInputStream.scaleTranslation(0.3)
                                .scaleRotation(0.2),
                        () -> swerveInputStream.scaleTranslation(0.8)
                                .scaleRotation(0.6)
                ));
                // Boost speed while holding right trigger
                driverXbox.rightTrigger().whileTrue(Commands.runEnd(
                        () -> swerveInputStream.scaleTranslation(1)
                                .scaleRotation(.75),
                        () -> swerveInputStream.scaleTranslation(.8)
                                .scaleRotation(.6)
                ));

                // Toggle to invert controls
                driverXbox.back().toggleOnTrue(Commands.runEnd(
                        () -> swerveInputStream.translationHeadingOffset(true),
                        () -> swerveInputStream.translationHeadingOffset(false)
                ));

                // Toggle field or robot relative speeds
                driverXbox.start().toggleOnTrue(Commands.runEnd(
                        () -> swerveInputStream.robotRelative(false)
                                .allianceRelativeControl(true),
                        () -> swerveInputStream.robotRelative(true)
                                .allianceRelativeControl(false)
                ));

                // Drive to reef
                driverXbox.a().whileTrue(swerveSubsystem.driveToReef(poseSelector));
                // Drive to station
                driverXbox.b().whileTrue(swerveSubsystem.driveToStation(poseSelector));
                // Drive to processor
                driverXbox.x().whileTrue(swerveSubsystem.driveToProcessor(poseSelector));
                // Drive into climb
                driverXbox.y().whileTrue(
                        swerveSubsystem.driveToCage(poseSelector)
                                .until(swerveSubsystem.atCage(poseSelector))
                                .andThen(swerveSubsystem.driveBackwards()
                                        .withTimeout(.6)));

                break;
            case DUAL_XBOX:
                swerveSubsystem.setDefaultCommand(Commands.run(Commands::none, swerveSubsystem));
                break;
            case SINGLE_STICK:
                swerveSubsystem.setDefaultCommand(Commands.run(Commands::none, swerveSubsystem));
                break;
            case DUAL_STICK:
                swerveSubsystem.setDefaultCommand(Commands.run(Commands::none, swerveSubsystem));
                break;
            case SINGLE_STICK_XBOX:
                swerveSubsystem.setDefaultCommand(Commands.run(Commands::none, swerveSubsystem));
                break;
            case DUAL_STICK_XBOX:
                swerveSubsystem.setDefaultCommand(Commands.run(Commands::none, swerveSubsystem));
                break;
        }
    }
}
