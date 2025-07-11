package frc.robot.utils.maplesim.opponents;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
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

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.maplesim.MapleConstants.PoseConstants.*;

public abstract class SmartOpponent extends SubsystemBase {

    protected Optional<Integer> id = Optional.empty();

    protected Optional<DriverStation.Alliance> alliance = Optional.empty();

    protected Optional<SelfControlledSwerveDriveSimulation> simulation = Optional.empty();

    protected Optional<States> currentState = Optional.empty();

    protected Optional<States> previousState = Optional.empty();

    protected boolean commandInProgress = false;

    protected Optional<Pose2d> startPose = Optional.empty();

    protected Optional<Pose2d> queeningPose = Optional.empty();

    protected Optional<StructPublisher<Pose2d>> posePublisher = Optional.empty();

    protected Optional<StringPublisher> statePublisher = Optional.empty();

    protected Optional<SendableChooser<Command>> behaviorChooser = Optional.empty();

    protected Optional<Integer> scoreTarget = Optional.empty();

    protected Optional<PPHolonomicDriveController> driveController = Optional.empty();

    protected Optional<Mass> opponentMassKG = Optional.empty();
    protected Optional<Double> opponentMOI = Optional.empty();
    protected Optional<Distance> opponentWheelRadius = Optional.empty();
    protected Optional<LinearVelocity> opponentDriveVelocity = Optional.empty();
    protected Optional<Double> opponentDriveCOF = Optional.empty();
    protected Optional<DCMotor> opponentDriveMotor = Optional.empty();
    protected Optional<Double> opponentDriveCurrentLimit = Optional.empty();
    protected Optional<Integer> opponentNumDriveMotors = Optional.empty();
    protected Optional<Distance> opponentTrackWidth = Optional.empty();

    // PathPlanner configuration
    protected Optional<RobotConfig> pathplannerConfig = Optional.empty();
    protected Optional<DriveTrainSimulationConfig> driveConfig = Optional.empty();

    /**
     * A default abstracted class for making smart opponent robots.
     * Smart Opponents run as a state machine.
     */
    public SmartOpponent() {
    }

    /**
     * @param id
     * @param alliance
     */
    public void setupOpponent(int id, DriverStation.Alliance alliance) {
        this.id = Optional.of(id);
        this.alliance = Optional.of(alliance);
        this.startPose = Optional.of(ROBOTS_STARTING_POSITIONS[id]);
        this.queeningPose = Optional.of(ROBOT_QUEENING_POSITIONS[id]);
        this.driveConfig = Optional.of(DriveTrainSimulationConfig.Default());
        this.simulation = Optional.of(
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                        driveConfig.get(),
                        queeningPose.get()
                )));
        this.driveController = Optional.of(
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0), new PIDConstants(5.0, 0.0)));
        if
        (
                opponentMassKG.isPresent() &&
                        opponentMOI.isPresent() &&
                        opponentWheelRadius.isPresent() &&
                        opponentDriveVelocity.isPresent() &&
                        opponentDriveCOF.isPresent() &&
                        opponentDriveMotor.isPresent() &&
                        opponentDriveCurrentLimit.isPresent() &&
                        opponentNumDriveMotors.isPresent() &&
                        opponentTrackWidth.isPresent()
        ) {
            this.pathplannerConfig = Optional.of(new RobotConfig(
                    opponentMassKG.get().in(Kilograms),
                    opponentMOI.get(),
                    new ModuleConfig(
                            opponentWheelRadius.get().in(Inches),
                            opponentDriveVelocity.get().in(MetersPerSecond),
                            opponentDriveCOF.get(),
                            opponentDriveMotor.get(),
                            opponentDriveCurrentLimit.get(),
                            opponentNumDriveMotors.get()),
                    opponentTrackWidth.get().in(Meters)
            ));
        } else {
            DriverStation.reportWarning("Pathplanner config not found, breaking is likely", false);
        }
        SimulatedArena.getInstance().addDriveTrainSimulation(
                this.simulation.get().getDriveTrainSimulation());
        this.posePublisher = Optional.of(
                NetworkTableInstance.getDefault()
                        .getStructTopic("SmartDashboard/MapleSim/SimulatedRobots/Poses/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + id + " Pose", Pose2d.struct).publish());
        this.statePublisher = Optional.of(
                NetworkTableInstance.getDefault()
                        .getStringTopic("SmartDashboard/MapleSim/SimulatedRobots/States/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + id + " Current State").publish());
        this.scoreTarget = Optional.of(1);
        setState(States.STANDBY);
        buildBehaviorChooser();
    }

    /**
     * @param newState
     */
    public void setState(States newState) {
        currentState = Optional.of(newState);
    }

    /**
     *
     */
    public void runStateCommand(States newState) {
        if (commandInProgress) return;

        Command stateCommand = switch (newState) {
            case STANDBY -> standbyCommand();
            case STARTING -> startingCommand();
            case COLLECT -> collectCommand();
            case SCORE -> scoreCommand();
            case JOYSTICK -> joystickCommand();
            case DEFEND -> defendCommand();
        };

        if (stateCommand != null) {
            commandInProgress = true;
            Optional<States> thisState = currentState; // States change during the state command.
            stateCommand
                    .finallyDo(() -> {
                        commandInProgress = false;
                        previousState = thisState;
                    })
                    .schedule();
        }
    }

    /**
     * Build the behavior chooser of this opponent robot and send it to the dashboard
     */
    public void buildBehaviorChooser() {
        if (simulation.isPresent() && queeningPose.isPresent()) {
            this.behaviorChooser = Optional.of(new SendableChooser<>());

            // Option to disable the robot
            behaviorChooser.get().setDefaultOption("Disable", Commands.runOnce(() -> setState(States.STANDBY))
                    .andThen(standbyCommand()));

            // Option to auto-cycle random
            behaviorChooser.get().addOption(
                    "Smart Cycle", Commands.runOnce(() -> setState(States.STARTING)));

            // Schedule the command when another behavior is selected
            behaviorChooser.get().onChange((Command::schedule));

            // Schedule the selected command when teleop starts
            RobotModeTriggers.teleop()
                    .onTrue(Commands.runOnce(() -> behaviorChooser.get().getSelected().schedule()));

            // Disable the robot when the user robot is disabled
            RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> setState(States.STANDBY)));

            SmartDashboard.putData("MapleSim/SimulatedRobots/Behaviors/ " + alliance + id + " Behavior", behaviorChooser.get());
        } else {
            DriverStation.reportWarning("No simulation found", false);
        }

    }

    @Override
    public void simulationPeriodic() {
        currentState.ifPresent(state -> {
            if (!commandInProgress && (previousState.isEmpty() || state != previousState.get())) {
                runStateCommand(state);
                previousState = Optional.of(state);
                commandInProgress = true;
            }
        });
        simulation.ifPresent(
                simulation -> posePublisher.ifPresent(
                        posePublisher -> posePublisher.set(simulation.getActualPoseInSimulationWorld())));
        currentState.ifPresent(
                state -> statePublisher.ifPresent(
                        statePublisher -> statePublisher.set(state.toString())));
    }


    /**
     *
     */
    public Command standbyCommand() {
        if (simulation.isPresent() && queeningPose.isPresent()) {
            return Commands.runOnce(() -> simulation.get().setSimulationWorldPose(queeningPose.get()), this)
                    .andThen(Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true);
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command startingCommand() {
        if (startPose.isPresent()) {
            return Commands.runOnce(() -> getSimulation().setSimulationWorldPose(startPose.get()))
                    .andThen(Commands.runOnce(() -> getSimulation().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true)
                    .andThen(() -> setState(States.COLLECT));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command collectCommand() {
        if (simulation.isPresent()) {
            return pathfindToPath(pathFromPose(getCollectPose()))
                    .andThen(() -> setState(States.SCORE));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command scoreCommand() {
        if (simulation.isPresent() && scoreTarget.isPresent()) {
            return pathfindToPath(pathFromPose(getScorePose()))
                    .andThen(scoreTarget.get() >= 12 ? algaeFeedShot() : coralFeedShot())
                    .andThen(() -> setState(States.COLLECT));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    public Command joystickCommand() {
        return Commands.runOnce(() -> DriverStation.reportWarning("No Joystick state implemented", false), this);
    }

    /**
     *
     */
    public Command joystickDrive(DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        if (simulation.isPresent()) {
            // Obtain chassis speeds from joystick input
            final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
                    leftY.getAsDouble() * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                    leftX.getAsDouble() * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                    rightX.getAsDouble() * simulation.get().maxAngularVelocity().in(RadiansPerSecond));

            // Obtain driverstation facing for opponent driver station
            final Supplier<Rotation2d> opponentDriverStationFacing = () ->
                    FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));
            return
                    Commands.run(() -> {
                                // Calculate field-centric speed from driverstation-centric speed
                                final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                        joystickSpeeds.get(),
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                                .plus(Rotation2d.fromDegrees(180)));
                                // Run the field-centric speed
                                simulation.get().runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
                            }, this)
                            // Before the command starts, reset the robot to a position inside the field
                            .beforeStarting(this::startingCommand);
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     * @return
     */
    public Command defendCommand() {
        return Commands.runOnce(() -> DriverStation.reportWarning("No defend state implemented", false), this);
    }

    /**
     * Follow path command for opponent robots
     */
    public Command followPath(PathPlannerPath path) {
        if (simulation.isPresent() &&
                driveController.isPresent() &&
                pathplannerConfig.isPresent()) {
            return new FollowPathCommand(
                    path, // Specify the path
                    // Provide actual robot pose in simulation, bypassing odometry error
                    simulation.get()::getActualPoseInSimulationWorld,
                    // Provide actual robot speed in simulation, bypassing encoder measurement error
                    simulation.get()::getActualSpeedsRobotRelative,
                    // Chassis speeds output
                    (speeds, feedforwards) ->
                            simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false),
                    driveController.get(), // Specify PID controller
                    pathplannerConfig.get(),       // Specify robot configuration
                    // Flip path based on alliance side
                    () -> DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .equals(DriverStation.Alliance.Red), this)
                    .until(() -> nearPose(path.getPathPoses().getLast(), Inches.of(6)));
        } else {
            return Commands.runOnce(() -> {
                DriverStation.reportWarning("No simulation found", false);
            });
        }
    }

    /**
     *
     */
    public Command pathfindToPath(PathPlannerPath path) {
        if (simulation.isPresent() &&
                driveController.isPresent() &&
                pathplannerConfig.isPresent()) {
            return new FollowPathCommand(
                    path, // Specify the path
                    // Provide actual robot pose in simulation, bypassing odometry error
                    simulation.get()::getActualPoseInSimulationWorld,
                    // Provide actual robot speed in simulation, bypassing encoder measurement error
                    simulation.get()::getActualSpeedsRobotRelative,
                    // Chassis speeds output
                    (speeds, feedforwards) ->
                            simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false),
                    driveController.get(), // Specify PID controller
                    pathplannerConfig.get(),       // Specify robot configuration
                    // Flip path based on alliance side
                    () -> DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .equals(DriverStation.Alliance.Red), this)
                    .until(() -> nearPose(path.getPathPoses().getLast(), Inches.of(6)));
        } else {
            return Commands.runOnce(() -> {
                DriverStation.reportWarning("No simulation found", false);
            }, this);
        }
    }

    /**
     *
     */
    public Command pathfindToPose(Pose2d pose) {
        return Commands.runOnce(() -> {
            DriverStation.reportWarning("pathFindToPose() not yet implemented", false);
        }, this);
    }

    /**
     * @param pose
     * @return
     */
    public PathPlannerPath pathFromPose(Pose2d pose) {
        Transform2d transform1 = new Transform2d(
                Units.inchesToMeters(-24),
                0,
                Rotation2d.kZero
        );
        Transform2d transform2 = new Transform2d(
                Units.inchesToMeters(-12),
                0,
                Rotation2d.kZero
        );
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                pose.plus(transform1),
                pose.plus(transform2),
                pose
        );
        return new PathPlannerPath(
                waypoints,
                getPathConstraints(),
                null,
                new GoalEndState(0.0,
                        pose.getRotation())
        );
    }

    /**
     *
     */
    public boolean nearPose(Pose2d pose, Distance maxDistance) {
        Translation2d robotTranslation = getSimulation().getActualPoseInSimulationWorld().getTranslation();
        Translation2d goalTranslation = pose.getTranslation();
        double distance = robotTranslation.getDistance(goalTranslation);

        return distance <= maxDistance.in(Meters);
    }

    /**
     * @return
     */
    public PathConstraints getPathConstraints() {
        double speedScalar = MathUtil.clamp(Math.random(), 0.7, 1); // Randomly vary speed for now.
        return new PathConstraints(
                15 * speedScalar,
                5 * speedScalar,
                Units.degreesToRadians(360) * speedScalar,
                Units.degreesToRadians(720) * speedScalar);
    }

    /**
     *
     */
    public Command coralFeedShot() {
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
    public Command algaeFeedShot() {
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
                                    });
                        }
                    });
        });
    }

    /**
     * @return
     */
    public Pose2d getCollectPose() {
        int station = ((int) Math.round(Math.random()));
        Pose2d stationPose = switch (station) {
            case 0 -> LEFT_STATION_POSE;
            case 1 -> RIGHT_STATION_POSE;
            default -> Pose2d.kZero;
        };
        return ifShouldFlip(stationPose);
    }

    /**
     * @return
     */
    public SelfControlledSwerveDriveSimulation getSimulation() {
        if (simulation.isPresent()) {
            return simulation.get();
        } else {
            DriverStation.reportWarning("No simulation found, causing a null pointer", false);
            return null;
        }
    }

    /**
     * @return
     */
    public Pose2d getScorePose() {
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
     * @param pose
     * @return
     */
    public Pose2d ifShouldFlip(Pose2d pose) {
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
         * This state puts the robot into the starting pose.
         */
        STARTING,
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
