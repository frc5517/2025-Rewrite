package frc.robot.utils.maplesim;


import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class SimulatedRobots extends SubsystemBase {
    /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[]
            {
                    new Pose2d(-6, 0, new Rotation2d()),
                    new Pose2d(-5, 0, new Rotation2d()),
                    new Pose2d(-4, 0, new Rotation2d()),
                    new Pose2d(-3, 0, new Rotation2d()),
                    new Pose2d(-2, 0, new Rotation2d())
            };
    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[]
            {
                    new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
                    new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
                    new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
                    new Pose2d(1.6, 6, new Rotation2d()),
                    new Pose2d(1.6, 4, new Rotation2d())
            };
    public static final SimulatedRobots[] instances = new SimulatedRobots[2]; // This should match the count of simulated robots starts at 1 not 0.
    private static final double opponentMassKG = 55;
    private static final double opponentMOI = 8;
    private static final double opponentWheelRadius = Units.inchesToMeters(2);
    private static final double opponentDriveVelocity = 8.5;
    private static final double opponentDriveCOF = 1.19;
    private static final DCMotor opponentDriveMotor = DCMotor.getNEO(1)
            .withReduction(8.14);
    // Game piece setup is below
    private static final double opponentDriveCurrentLimit = 40;
    private static final int opponentNumDriveMotors = 1;
    private static final double opponentTrackWidth = Units.inchesToMeters(23);
    private static final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default();
    // PathPlanner configuration
    private static final RobotConfig pathplannerConfig = new RobotConfig(
            opponentMassKG,
            opponentMOI,
            new ModuleConfig(
                    opponentWheelRadius,
                    opponentDriveVelocity,
                    opponentDriveCOF,
                    opponentDriveMotor,
                    opponentDriveCurrentLimit,
                    opponentNumDriveMotors),
            opponentTrackWidth
    );
    // PathPlanner PID settings
    private final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));
    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;
    private final String alliance;
    StructPublisher<Pose2d> posePublisher;

    public SimulatedRobots(int id, String alliance) {
        this.id = id;
        this.alliance = alliance;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        this.driveSimulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                driveConfig,
                queeningPose
        ));

        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/MapleSim/SimulatedRobots/Poses/ " + alliance + id + " Pose", Pose2d.struct).publish();

        SimulatedArena.getInstance().addDriveTrainSimulation(
                driveSimulation.getDriveTrainSimulation()
        );
    }

    public static void startOpponentRobotSimulations() {
        try {
            instances[0] = new SimulatedRobots(0, "Red Alliance ");
            instances[0].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("Opponent Left Cycle 0"),
                    instances[0].coralFeedShot(),
                    PathPlannerPath.fromPathFile("Opponent Left Cycle Back 0"),
                    Commands.none(),
                    new CommandXboxController(3));

            instances[1] = new SimulatedRobots(1, "Red Alliance ");
            instances[1].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("Opponent Right Cycle 1"),
                    instances[1].algaeFeedShot(DriverStation.Alliance.Red),
                    PathPlannerPath.fromPathFile("Opponent Right Cycle Back 1"),
                    Commands.none(),
                    new CommandXboxController(4));
            // create more opponent robots if you need
        } catch (Exception e) {
            DriverStation.reportError("Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
        }
    }

    @Override
    public void periodic() {
        posePublisher.set(driveSimulation.getActualPoseInSimulationWorld());
    }

    /**
     * Joystick drive command for opponent robots
     */
    private Command joystickDrive(CommandXboxController joystick) {
        // Obtain chassis speeds from joystick input
        final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
                -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
                -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
                -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond));


        // Obtain driverstation facing for opponent driver station
        final Supplier<Rotation2d> opponentDriverStationFacing = () ->
                FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));

        return Commands.run(() -> {
                    // Calculate field-centric speed from driverstation-centric speed
                    final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                            joystickSpeeds.get(),
                            FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                    .plus(Rotation2d.fromDegrees(180)));
                    // Run the field-centric speed
                    driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
                }, this)
                // Before the command starts, reset the robot to a position inside the field
                .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id])));
    }

    /**
     * Follow path command for opponent robots
     */
    private Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path, // Specify the path
                // Provide actual robot pose in simulation, bypassing odometry error
                driveSimulation::getActualPoseInSimulationWorld,
                // Provide actual robot speed in simulation, bypassing encoder measurement error
                driveSimulation::getActualSpeedsRobotRelative,
                // Chassis speeds output
                (speeds, feedforwards) ->
                        driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
                driveController, // Specify PID controller
                pathplannerConfig,       // Specify robot configuration
                // Flip path based on alliance side
                () -> DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        .equals(DriverStation.Alliance.Red),
                this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
        );
    }

    /**
     * Build the behavior chooser of this opponent robot and send it to the dashboard
     */
    public void buildBehaviorChooser(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1,
            CommandXboxController joystick) {
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                        .ignoringDisable(true);

        // Option to disable the robot
        behaviorChooser.setDefaultOption("Disable", disable.get());

        // Option to auto-cycle the robot
        behaviorChooser.addOption(
                "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

        // Option to manually control the robot with a joystick
        behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

        // Schedule the command when another behavior is selected
        behaviorChooser.onChange((Command::schedule));

        // Schedule the selected command when teleop starts
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

        // Disable the robot when the user robot is disabled
        RobotModeTriggers.disabled().onTrue(disable.get());

        SmartDashboard.putData("MapleSim/SimulatedRobots/Behaviors/ " + alliance + id + " Behavior", behaviorChooser);
    }

    /**
     * Get the command to auto-cycle the robot relatively
     */
    private Command getAutoCycleCommand(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1) {
        final SequentialCommandGroup cycle = new SequentialCommandGroup();
        final Pose2d startingPose = new Pose2d(
                segment0.getStartingDifferentialPose().getTranslation(),
                segment0.getIdealStartingState().rotation());

        cycle.addCommands(
                opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
        cycle.addCommands(
                opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

        return cycle.repeatedly()
                .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
    }

    /**
     * @return A command to be used by simulated robots to launch coral according to set values.
     */
    private Command coralFeedShot() {
        // Setup to match kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(.85);
        LinearVelocity shootSpeed = MetersPerSecond.of(1);
        Angle shootAngle = Degrees.of(-15);
        Translation2d shootOnBotPosition = new Translation2d(
                0.5,
                0);

        return runOnce(() -> {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (x, y) (meters) on the robot
                            shootOnBotPosition,
                            // Obtain robot speed from drive simulation
                            driveSimulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            shootHeight,
                            // The initial speed of the coral
                            shootSpeed,
                            // The coral is ejected at a 35-degree slope
                            shootAngle));
        });
    }

    /**
     * @return A command to be used by simulated robots to launch algae according to set values.
     */
    private Command algaeFeedShot(DriverStation.Alliance alliance) {
        // Algae settings
        Distance shootHeight = Meters.of(3);
        LinearVelocity shootSpeed = MetersPerSecond.of(3);
        Angle shootAngle = Degrees.of(-25);
        Translation2d shootOnBotPosition = new Translation2d(
                0,
                0);

        return runOnce(() ->
        {
            GamePieceProjectile algae = new ReefscapeAlgaeOnFly(
                    // Obtain robot position from drive simulation
                    driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                    // The scoring mechanism is installed at (x, y) (meters) on the robot
                    shootOnBotPosition,
                    // Obtain robot speed from drive simulation
                    driveSimulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    // Obtain robot facing from drive simulation
                    driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                    // The height at which the coral is ejected
                    shootHeight,
                    // The initial speed of the coral
                    shootSpeed,
                    // The coral is ejected at a 35-degree slope
                    shootAngle);
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(algae);
            if (algae.willHitTarget() | algae.hasHitTarget()) {
                MapleSim.addAlgaeToScore(alliance, 1);
            }
        });

    }
}

