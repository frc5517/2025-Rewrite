package frc.robot.utils.maplesim.opponents;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.utils.maplesim.MapleSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.maplesim.MapleConstants.PoseConstants.*;

public abstract class SmartOpponent extends SubsystemBase {

    private Optional<Integer> id = Optional.empty();

    private Optional<DriverStation.Alliance> alliance = Optional.empty();

    private Optional<SelfControlledSwerveDriveSimulation> simulation = Optional.empty();

    private Optional<States> currentState = Optional.empty();

    private Optional<Pose2d> startPose = Optional.empty();

    private Optional<Pose2d> queeningPose = Optional.empty();

    private Optional<StructPublisher<Pose2d>> posePublisher = Optional.empty();

    private Optional<SendableChooser<Command>> behaviorChooser = Optional.empty();

    private Optional<Integer> scoreTarget = Optional.empty();

    private static final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(5.0, 0.05));
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
    private static final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default();

    /**
     * A default abstracted class for making smart opponent robots.
     * Smart Opponents run as a state machine.
     */
    public SmartOpponent()
    {

    }

    /**
     *
     * @param id
     * @param alliance
     */
    public void setupOpponent(int id, DriverStation.Alliance alliance) {
        this.id = Optional.of(id);
        this.alliance = Optional.of(alliance);
        this.startPose = Optional.of(ROBOTS_STARTING_POSITIONS[id]);
        this.queeningPose = Optional.of(ROBOT_QUEENING_POSITIONS[id]);
        this.simulation = Optional.of(
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                        driveConfig,
                        queeningPose.get()
                )));

        this.posePublisher = Optional.of(
                NetworkTableInstance.getDefault()
                        .getStructTopic("SmartDashboard/MapleSim/SimulatedRobots/Poses/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + id + " Pose", Pose2d.struct).publish());
    }

    /**
     *
     * @return
     */
    private Command setStandbyState() {
        return Commands.runOnce(() -> currentState.ifPresentOrElse(
                        currentState -> setState(States.STANDBY),
                        () -> {
                            DriverStation.reportWarning("Current state not found", false);
                        }))
                .ignoringDisable(true);
    }

    /**
     *
     * @return
     */
    private Command setCollectState() {
        return Commands.runOnce(() -> currentState.ifPresentOrElse(
                        currentState -> setState(States.COLLECT),
                        () -> {
                            DriverStation.reportWarning("Current state not found", false);
                        }))
                .ignoringDisable(true);
    }

    /**
     *
     * @param state
     */
    private void setState(States state) {
        currentState = Optional.of(state);
    }

    /**
     * Build the behavior chooser of this opponent robot and send it to the dashboard
     */
    public void buildBehaviorChooser() {
        if (simulation.isPresent() && queeningPose.isPresent()) {
            this.behaviorChooser = Optional.of(new SendableChooser<>());

            // Option to disable the robot
            behaviorChooser.get().setDefaultOption("Disable", setStandbyState());

            // Option to auto-cycle random
            behaviorChooser.get().addOption(
                    "Smart Cycle", setCollectState());

            // TODO needs added during withOperatorControl
//            // Option to manually control the robot with a joystick
//            behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

            // Schedule the command when another behavior is selected
            behaviorChooser.get().onChange((Command::schedule));

            // Schedule the selected command when teleop starts
            RobotModeTriggers.teleop()
                    .onTrue(Commands.runOnce(() -> behaviorChooser.get().getSelected().schedule()));

            // Disable the robot when the user robot is disabled
            RobotModeTriggers.disabled().onTrue(setStandbyState());

            SmartDashboard.putData("MapleSim/SimulatedRobots/Behaviors/ " + alliance + id + " Behavior", behaviorChooser.get());
        } else {
            DriverStation.reportWarning("No simulation found", false);
        }

    }


    /**
     *
     */
    public void simIterate()
    {
        currentState.ifPresent(
                state -> {
                    switch (state) {
                        case STANDBY: Commands.none();
                        break;
                        case COLLECT: Commands.none();
                        break;
                        case SCORE: Commands.none();
                        break;
                        case JOYSTICK: Commands.none();
                        break;
                        case DEFEND: Commands.none();
                        break;
                    }
                }
        );

        simulation.ifPresent(
                simulation -> posePublisher.ifPresent(
                        posePublisher -> posePublisher.set(simulation.getActualPoseInSimulationWorld())));
    }

    /**
     * Follow path command for opponent robots
     */
    private Command followPath(PathPlannerPath path) {
        if (simulation.isPresent()) {
            return new FollowPathCommand(
                    path, // Specify the path
                    // Provide actual robot pose in simulation, bypassing odometry error
                    simulation.get()::getActualPoseInSimulationWorld,
                    // Provide actual robot speed in simulation, bypassing encoder measurement error
                    simulation.get()::getActualSpeedsRobotRelative,
                    // Chassis speeds output
                    (speeds, feedforwards) ->
                            simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false),
                    driveController, // Specify PID controller
                    pathplannerConfig,       // Specify robot configuration
                    // Flip path based on alliance side
                    () -> DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .equals(DriverStation.Alliance.Red),
                    this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
            );
        } else {
            DriverStation.reportError("No simulation found", false);
            return Commands.none();
        }

    }


    /**
     *
     */
    private void pathfindToPath()
    {

    }

    /**
     *
     */
    private void pathfindToPose()
    {

    }

    /**
     *
     */
    private void pathfindToPathFromPose()
    {

    }

    /**
     * TODO
     * needs .withOperatorControls(DoubleSupplier, DoubleSupplier, Trigger, ect.) on classes that extend this class
     */
    private Command operatorControl(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX)
    {
        if (simulation.isPresent()) {
            // Obtain chassis speeds from joystick input
            final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
                    leftY.getAsDouble() * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                    leftX.getAsDouble() * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                    rightX.getAsDouble() * simulation.get().maxAngularVelocity().in(RadiansPerSecond));


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
                        simulation.get().runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
                    }, this)
                    // Before the command starts, reset the robot to a position inside the field
                    .beforeStarting(() -> id.ifPresent(
                            id -> simulation.get().setSimulationWorldPose(
                                    FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id]))
                    ));
        } else {
            DriverStation.reportWarning("No simulation found", false);
            return Commands.none();
        }
    }

    /**
     *
     */
    private Command coralFeedShot()
    {
        // Setup to match the 2025 kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(.85);
        LinearVelocity shootSpeed = MetersPerSecond.of(1);
        Angle shootAngle = Degrees.of(-15);
        Translation2d shootOnBotPosition = new Translation2d(
                0.5,
                0);

        return runOnce(() ->
        {
            simulation.ifPresent(
                    simulation ->
                    {
                        SimulatedArena.getInstance()
                                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                        // Obtain robot position from drive simulation
                                        simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                                        // The scoring mechanism is installed at (x, y) (meters) on the robot
                                        shootOnBotPosition,
                                        // Obtain robot speed from drive simulation
                                        simulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                        // Obtain robot facing from drive simulation
                                        simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                                        // The height at which the coral is ejected
                                        shootHeight,
                                        // The initial speed of the coral
                                        shootSpeed,
                                        // The coral is ejected at a 35-degree slope
                                        shootAngle));
                    });
        });
    }

    /**
     * @return
     */
    private Command algaeFeedShot()
    {
        // Algae settings
        Distance shootHeight = Meters.of(3);
        LinearVelocity shootSpeed = MetersPerSecond.of(3);
        Angle shootAngle = Degrees.of(-25);
        Translation2d shootOnBotPosition = new Translation2d(
                0,
                0);

        return runOnce(() ->
        {
            simulation.ifPresent(
                    simulation -> {
                        GamePieceProjectile algae = new ReefscapeAlgaeOnFly(
                                // Obtain robot position from drive simulation
                                simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getTranslation(),
                                // The scoring mechanism is installed at (x, y) (meters) on the robot
                                shootOnBotPosition,
                                // Obtain robot speed from drive simulation
                                simulation.getDriveTrainSimulation().getDriveTrainSimulatedChassisSpeedsRobotRelative(),
                                // Obtain robot facing from drive simulation
                                simulation.getDriveTrainSimulation().getSimulatedDriveTrainPose().getRotation(),
                                // The height at which the coral is ejected
                                shootHeight,
                                // The initial speed of the coral
                                shootSpeed,
                                // The coral is ejected at a 35-degree slope
                                shootAngle);
                        SimulatedArena.getInstance()
                                .addGamePieceProjectile(algae);

                        if (algae.willHitTarget() | algae.hasHitTarget()) {
                            alliance.ifPresent(
                                    alliance -> {
                                        MapleSim.addAlgaeToScore(alliance, 1);
                                    });}
                    });
        });
    }

    /**
     *
     * @return
     */
    private Pose2d getCollectPose() {
        int station = ((int) Math.round(Math.random()));
        Pose2d stationPose = switch (station) {
            case 0 -> LEFT_STATION_POSE;
            case 1 -> RIGHT_STATION_POSE;
            default -> Pose2d.kZero;
        };
        return ifShouldFlip(stationPose);
    }

    /**
     *
     * @return
     */
    private Pose2d getScorePose() {
        this.scoreTarget = Optional.of(((int) Math.round(Math.random() * 17)));
        Pose2d targetPose = switch (scoreTarget.get()) {
            case 0 -> REEF_SOUTH_LEFT_POSE;
            case 1 -> REEF_SOUTH_RIGHT_POSE;
            case 2 -> REEF_SOUTHEAST_LEFT_POSE;
            case 3 -> REEF_SOUTHEAST_RIGHT_POSE;
            case 4 -> REEF_NORTHEAST_LEFT_POSE;
            case 5 -> REEF_NORTHEAST_RIGHT_POSE;
            case 6 -> REEF_NORTH_LEFT_POSE;
            case 7 -> REEF_NORTH_RIGHT_POSE;
            case 8 -> REEF_NORTHWEST_LEFT_POSE;
            case 9 -> REEF_NORTHWEST_RIGHT_POSE;
            case 10 -> REEF_SOUTHWEST_LEFT_POSE;
            case 11 -> REEF_SOUTHWEST_RIGHT_POSE;
            case 13 -> BARGE_NET_POSE;
            case 14 -> BARGE_NET_POSE;
            case 15 -> BARGE_NET_POSE;
            case 16 -> BARGE_NET_POSE;
            case 17 -> BARGE_NET_POSE;
            default -> Pose2d.kZero;
        };
        return ifShouldFlip(targetPose);
    }

    /**
     *
     * @param pose
     * @return
     */
    private Pose2d ifShouldFlip(Pose2d pose) {
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return pose;
            } else {
                return new Pose2d(
                        FieldMirroringUtils.flip(pose.getTranslation()),
                        FieldMirroringUtils.flip(pose.getRotation()));
            }
        } else {
            return pose;
        }
    }

    /**
     * The current state the opponent should be in. Some states may not be used.
     */
    public enum States {
        /**
         * This state puts the {@link SmartOpponent} in it's queening pose.
         */
        STANDBY,
        /**
         * This state has the opponent decide where then goes to collect a game piece.
         */
        COLLECT,
        /**
         * This state has the opponent decide where then goes and attempts to score a game piece.
         * This typically ignores whether the opponent has properly collected a piece.
         */
        SCORE,
        /**
         * This state lets the opponent be controlled by a controller once setup.
         */
        JOYSTICK,
        /**
         * This state has the opponent defend.
         */
        DEFEND
    }

}
