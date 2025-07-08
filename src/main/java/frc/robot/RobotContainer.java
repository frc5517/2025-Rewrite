// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.utils.maplesim.MapleSim;
import swervelib.SwerveInputStream;

import java.io.File;


public class RobotContainer {
    private final SendableChooser<BindingsSelector.BindingType> bindingSendable = new SendableChooser<>();
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseSelector poseSelector = new PoseSelector(swerve);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooter = new IntakeShooterSubsystem(
            swerve, elevator, arm);
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
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        bindingsSendableInit();
        configureBindings();
        setupAutonomous();
    }

    private void bindingsSendableInit() {
        bindingSendable.addOption("Single Xbox", BindingsSelector.BindingType.SINGLE_XBOX);
        bindingSendable.addOption("Dual Xbox", BindingsSelector.BindingType.DUAL_XBOX);
        bindingSendable.addOption("Single Stick", BindingsSelector.BindingType.SINGLE_STICK);
        bindingSendable.addOption("Dual Stick", BindingsSelector.BindingType.DUAL_STICK);
        bindingSendable.addOption("Single Stick and Xbox", BindingsSelector.BindingType.SINGLE_STICK_XBOX);
        bindingSendable.addOption("Dual Stick and Xbox", BindingsSelector.BindingType.DUAL_STICK_XBOX);
        bindingSendable.setDefaultOption("Testing", BindingsSelector.BindingType.TESTING);
        SmartDashboard.putData("Control Type", bindingSendable);
    }

//
//  Autonomous stuff
//

    public void configureBindings() {
        // Always on controls
        // TO-DO: add always on to force update controls.

        switch (bindingSendable.getSelected()) {
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

        isTesting.and(driverXbox.start()).onTrue(Commands.runOnce(() -> MapleSim.addCoralAllStations(false)));
        isTesting.and(driverXbox.back()).onTrue(Commands.runOnce(MapleSim::clearMatchData));

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

    public void setupAutonomous() {
//        NamedCommands.registerCommand("structuresToL1", superStructure.structureToL1());
//        NamedCommands.registerCommand("structuresToL2", superStructure.structureToL2());
//        NamedCommands.registerCommand("structuresToL3", superStructure.structureToL3());
//        NamedCommands.registerCommand("structuresToL4", superStructure.structureToL4());
//        NamedCommands.registerCommand("intake", intakeShooterSubsystem.intake());
//        NamedCommands.registerCommand("shoot", intakeShooterSubsystem.shoot());
//
//        NamedCommands.registerCommand("enablePID", superStructure.enablePID()
//                .andThen(superStructure.updateStowCommand()));
//        NamedCommands.registerCommand("disablePID", superStructure.disablePID()
//                .andThen(superStructure.updateStowCommand()));
//
//        NamedCommands.registerCommand("getCoral", superStructure.getCoral());
//        NamedCommands.registerCommand("scoreL1", superStructure.scoreL1());
//        NamedCommands.registerCommand("scoreL2", superStructure.scoreL2());
//        NamedCommands.registerCommand("scoreL3", superStructure.scoreL3());
//        NamedCommands.registerCommand("scoreL4", superStructure.scoreL4());

        NamedCommands.registerCommand("cycleStationUp", Commands.runOnce(poseSelector::cycleStationSlotUp));
        NamedCommands.registerCommand("cycleStationDown", Commands.runOnce(poseSelector::cycleStationSlotDown));
        NamedCommands.registerCommand("selectSlot1", Commands.runOnce(poseSelector::selectSlot1));
        NamedCommands.registerCommand("selectSlot2", Commands.runOnce(poseSelector::selectSlot2));
        NamedCommands.registerCommand("selectSlot3", Commands.runOnce(poseSelector::selectSlot3));

        NamedCommands.registerCommand("selectSouth", Commands.runOnce(poseSelector::selectSouth));
        NamedCommands.registerCommand("selectSoutheast", Commands.runOnce(poseSelector::selectSouthEast));
        NamedCommands.registerCommand("selectSouthwest", Commands.runOnce(poseSelector::selectSouthWest));
        NamedCommands.registerCommand("selectNorth", Commands.runOnce(poseSelector::selectNorth));
        NamedCommands.registerCommand("selectNortheast", Commands.runOnce(poseSelector::selectNorthEast));
        NamedCommands.registerCommand("selectNorthwest", Commands.runOnce(poseSelector::selectNorthWest));
        NamedCommands.registerCommand("selectLeft", Commands.runOnce(poseSelector::selectLeft));
        NamedCommands.registerCommand("selectRight", Commands.runOnce(poseSelector::selectRight));
//        NamedCommands.registerCommand("driveToReef", Commands.defer(() -> swerve.driveToPose(poseSelector::flippedReefPose, elevator.scaleForDrive(.8)), Set.of(swerve)));
//        NamedCommands.registerCommand("driveToStation", Commands.defer(() -> swerve.driveToPose(poseSelector::flippedStationPose, elevatorSubsystem.scaleForDrive(.8)), Set.of(drivebase)));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
