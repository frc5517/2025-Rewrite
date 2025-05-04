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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
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

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseSelector poseSelector = new PoseSelector(swerve);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooter = new IntakeShooterSubsystem(
            swerve, elevator, arm);

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
        bindingSendable.addOption("Single Xbox", BindingsSelector.BindingType.SINGLE_XBOX);
        bindingSendable.addOption("Dual Xbox", BindingsSelector.BindingType.DUAL_XBOX);
        bindingSendable.addOption("Single Stick", BindingsSelector.BindingType.SINGLE_STICK);
        bindingSendable.addOption("Dual Stick", BindingsSelector.BindingType.DUAL_STICK);
        bindingSendable.addOption("Single Stick and Xbox", BindingsSelector.BindingType.SINGLE_STICK_XBOX);
        bindingSendable.addOption("Dual Stick and Xbox", BindingsSelector.BindingType.DUAL_STICK_XBOX);
        bindingSendable.setDefaultOption("Testing", BindingsSelector.BindingType.TESTING);
        SmartDashboard.putData("Control Type", bindingSendable);
    }

    private final SwerveInputStream xboxStream = SwerveInputStream.of(swerve.getSwerveDrive(),
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
    Command driveXboxCommand = swerve.driveFieldOriented(xboxStream);
    
    public void configureBindings()
    {
        // Always on controls
        // TO-DO: add always on to force update controls.

        switch(bindingSendable.getSelected()) {
            case SINGLE_XBOX:
                swerve.setDefaultCommand(driveXboxCommand);
                break;
            case DUAL_XBOX:
                swerve.setDefaultCommand(Commands.run(Commands::none, swerve));
                break;
            case SINGLE_STICK:
                swerve.setDefaultCommand(Commands.run(Commands::none, swerve));
                break;
            case DUAL_STICK:
                swerve.setDefaultCommand(Commands.run(Commands::none, swerve));
                break;
            case SINGLE_STICK_XBOX:
                swerve.setDefaultCommand(Commands.run(Commands::none, swerve));
                break;
            case DUAL_STICK_XBOX:
                swerve.setDefaultCommand(Commands.run(Commands::none, swerve));
                break;
            case TESTING:
                swerve.setDefaultCommand(driveXboxCommand);
                break;
        }

        /*
            Single Xbox Bindings
         */

        Trigger isSingleXbox = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.SINGLE_XBOX);

        isSingleXbox.and(driverXbox.povUp()).onTrue(Commands.runOnce(poseSelector::selectNorth));
        isSingleXbox.and(driverXbox.povUpRight()).onTrue(Commands.runOnce(poseSelector::selectNorthEast));
        isSingleXbox.and(driverXbox.povDownRight()).onTrue(Commands.runOnce(poseSelector::selectSouthEast));
        isSingleXbox.and(driverXbox.povDown()).onTrue(Commands.runOnce(poseSelector::selectSouth));
        isSingleXbox.and(driverXbox.povDownLeft()).onTrue(Commands.runOnce(poseSelector::selectSouthWest));
        isSingleXbox.and(driverXbox.povUpLeft()).onTrue(Commands.runOnce(poseSelector::selectNorthWest));

        // Choose left or right pose, used in station and reef
        isSingleXbox.and(driverXbox.povRight()).onTrue(Commands.runOnce(poseSelector::selectRight));
        isSingleXbox.and(driverXbox.povLeft()).onTrue(Commands.runOnce(poseSelector::selectLeft));

        // Cycle cage and station poses
        isSingleXbox.and(driverXbox.leftBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotDown));
        isSingleXbox.and(driverXbox.rightBumper()).onTrue(Commands.runOnce(poseSelector::cycleStationSlotUp));

        // Slow speed while holding left trigger
        isSingleXbox.and(driverXbox.leftTrigger()).whileTrue(Commands.runEnd(
                () -> xboxStream.scaleTranslation(0.3)
                        .scaleRotation(0.2),
                () -> xboxStream.scaleTranslation(0.8)
                        .scaleRotation(0.6)
        ));
        // Boost speed while holding right trigger
        isSingleXbox.and(driverXbox.rightTrigger()).whileTrue(Commands.runEnd(
                () -> xboxStream.scaleTranslation(1)
                        .scaleRotation(.75),
                () -> xboxStream.scaleTranslation(.8)
                        .scaleRotation(.6)
        ));

        // Toggle to invert controls
        isSingleXbox.and(driverXbox.back()).toggleOnTrue(Commands.runEnd(
                () -> xboxStream.translationHeadingOffset(true),
                () -> xboxStream.translationHeadingOffset(false)
        ));

        // Toggle field or robot relative speeds
        isSingleXbox.and(driverXbox.start()).toggleOnTrue(Commands.runEnd(
                () -> xboxStream.robotRelative(false)
                        .allianceRelativeControl(true),
                () -> xboxStream.robotRelative(true)
                        .allianceRelativeControl(false)
        ));

        // Drive to reef
        isSingleXbox.and(driverXbox.a()).whileTrue(swerve.driveToReef(poseSelector));
        // Drive to station
        isSingleXbox.and(driverXbox.b()).whileTrue(swerve.driveToStation(poseSelector));
        // Drive to processor
        isSingleXbox.and(driverXbox.x()).whileTrue(swerve.driveToProcessor(poseSelector));
        // Drive into climb
        isSingleXbox.and(driverXbox.y()).whileTrue(
                swerve.driveToCage(poseSelector)
                        .until(swerve.atCage(poseSelector))
                        .andThen(swerve.driveBackwards()
                                .withTimeout(.6)));


        /*
            Test Bindings
         */
        Trigger isTesting = new Trigger(() -> bindingSendable.getSelected() == BindingsSelector.BindingType.TESTING);

        isTesting.and(driverXbox.a()).whileTrue(arm.toL1().alongWith(elevator.toL1()));
        isTesting.and(driverXbox.b()).whileTrue(arm.toL2());
        isTesting.and(driverXbox.x()).whileTrue(arm.toL3());
        isTesting.and(driverXbox.y()).whileTrue(arm.toL4());
        isTesting.and(driverXbox.povUp()).whileTrue(arm.goUp());
        isTesting.and(driverXbox.povDown()).whileTrue(arm.goDown());
        isTesting.and(driverXbox.start()).whileTrue(arm.sysId());

        isTesting.and(driverXbox.leftBumper()).whileTrue(intakeShooter.intake());
        isTesting.and(driverXbox.rightBumper()).whileTrue(intakeShooter.shoot());

    }
}
