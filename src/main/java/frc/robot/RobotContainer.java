// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.utils.maplesim.opponents.reefscape.kitbot.KitBot;
import frc.robot.utils.maplesim.opponents.reefscape.kitbotpro.KitBotPro;
import frc.robot.utils.robot.InputStructure;
import frc.robot.utils.robot.PoseSelector;

import java.io.File;

public class RobotContainer {
    // Initialize the swerve subsystem.
    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    // Initialize the pose selector.
    private final PoseSelector poseSelector = new PoseSelector(swerve);
    // Initialize the elevator subsystem.
    private final Elevator elevator = new Elevator();
    // Initialize the arm subsystem.
    private final Arm arm = new Arm();
    // Initialize the LED subsystem.
    private final AddressableLEDSubsystem led = new AddressableLEDSubsystem();
    // Initialize the intake shooter subsystem.
    private final IntakeShooter intakeShooter = new IntakeShooter(
            swerve, elevator, arm);
    // Initialize the control superstructure, this controls all subsystems together.
    private final ControlStructure structure = new ControlStructure(
            swerve, poseSelector, arm, elevator, intakeShooter, led);
    // Initialize the bindings selector class.
    private final InputStructure bindings = new InputStructure(
            swerve, elevator, arm, intakeShooter, structure, poseSelector);
    // Initialize the autonomous selector.
    private SendableChooser<Command> autoChooser;

    /**
     * Called once on robot startup.
     */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        bindings.init();
        setupAutonomous();
        initOpponentSim();
    }

    /**
     * Initializes opponent sim if the robot is in simulation.
     */
    public void initOpponentSim() {
        // If simulation setup opponent sim.
        if (RobotBase.isSimulation()) {
            // Simulated Opponents
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
    }

    /**
     * Initializes the autonomous selector from pathplanner.
     */
    public void setupAutonomous() {
        // Named Commands go here
        //NamedCommands.registerCommand("GUI NAME", theCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    /**
     * Gets the selected autonomous command.
     *
     * @return the selected {@link Command}.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
