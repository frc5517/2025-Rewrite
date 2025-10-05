package frc.robot.utils.maplesim.opponents;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.maplesim.MapleSim;
import frc.robot.utils.maplesim.opponents.pathfinding.MapleADStar;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

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

    protected Optional<Trigger> isJoystick = Optional.empty();

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
    protected Optional<Object> joystick = Optional.empty();

    // PathPlanner configuration
    protected Optional<RobotConfig> pathplannerConfig = Optional.empty();
    protected Optional<DriveTrainSimulationConfig> driveConfig = Optional.empty();
    protected Optional<MapleADStar> mapleADStar = Optional.empty();
    // Static settings
    protected Double joystickDeadzone = 0.1;
    protected Distance driveToPoseCollectTolerance = Inches.of(8);
    protected Distance driveToPoseScoreTolerance = Inches.of(3);
    protected String robotName = "Smart Opponent";
    protected boolean isColliding = false;
    protected Debouncer collisionDebouncer = new Debouncer(1, Debouncer.DebounceType.kRising);
    protected double collisionFactor = 0;

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
        this.isJoystick = Optional.of(new Trigger(() -> currentState.get() == States.JOYSTICK));
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
                                + robotName + " " + id + " Pose", Pose2d.struct).publish());
        this.statePublisher = Optional.of(
                NetworkTableInstance.getDefault()
                        .getStringTopic("SmartDashboard/MapleSim/SimulatedRobots/States/ "
                                + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                                + robotName + " " + id + " Current State").publish());
        this.scoreTarget = Optional.of(1);
        this.mapleADStar = Optional.of(new MapleADStar());
        setState(States.STANDBY);
        buildBehaviorChooser(id, alliance);
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
                    }).schedule();
        }
    }

    /**
     * Build the behavior chooser of this opponent robot and send it to the dashboard
     */
    public void buildBehaviorChooser(int id, DriverStation.Alliance alliance) {
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
            RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> setState(States.STANDBY))
                    .andThen(standbyCommand()));

            SmartDashboard.putData("MapleSim/SimulatedRobots/Behaviors/ "
                    + (alliance.equals(DriverStation.Alliance.Red) ? "Red Alliance " : "Blue Alliance ")
                    + robotName + " " + id + " Behavior", behaviorChooser.get());
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
        detectCollision();
    }

    protected void detectCollision() {
        simulation.ifPresent(selfControlledSwerveDriveSimulation -> collisionFactor =
                LinearFilter.movingAverage(50).calculate(Math.abs(
                        (Arrays.stream(selfControlledSwerveDriveSimulation.getDriveTrainSimulation().getModules()).findFirst().get().getDriveMotorSupplyCurrent()).in(Amps)
                                - Arrays.stream(selfControlledSwerveDriveSimulation.getDriveTrainSimulation().getModules()).findAny().get().getDriveMotorAppliedVoltage().in(Volts))));
        isColliding = collisionDebouncer.calculate(MathUtil.isNear(0.6, collisionFactor, 0.3));
    }

    /**
     *
     */
    public Command standbyCommand() {
        if (simulation.isPresent() && queeningPose.isPresent()) {
            return Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this)
                    .andThen(Commands.runOnce(() -> simulation.get().setSimulationWorldPose(queeningPose.get()), this))
                    .ignoringDisable(true);
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command startingCommand() {
        if (simulation.isPresent() && startPose.isPresent()) {
            return Commands.runOnce(() -> simulation.get().setSimulationWorldPose(startPose.get()))
                    .andThen(Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true)
                    .andThen(Commands.waitSeconds(1))
                    .andThen(() -> setState(States.COLLECT));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command collectCommand() {
        if (simulation.isPresent() && currentState.isPresent()) {
            return (pathfindCommand(getCollectPose(), driveToPoseCollectTolerance).withTimeout(7)
                    .andThen(Commands.waitSeconds(0.5)))
                    .until(() -> isColliding)
                    .andThen(() -> setState(States.SCORE));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     *
     */
    public Command scoreCommand() {
        if (simulation.isPresent() && scoreTarget.isPresent() && mapleADStar.isPresent() && currentState.isPresent()) {
            return
                    (pathfindCommand(getScorePose(), driveToPoseScoreTolerance).withTimeout(7)
                            .andThen(scoreTarget.get() >= 12 ? algaeFeedShotCommand() : coralFeedShotCommand())
                            .andThen(Commands.waitSeconds(0.5)))
                            .until(() -> isColliding)
                            .andThen(() -> setState(States.COLLECT));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     * @return
     */
    public Command joystickCommand() {
        if (joystick.isPresent() && joystick.get() instanceof CommandXboxController) {
            return joystickDrive();
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No Joystick found, use .withJoystick() on SmartOpponent instance", false), this);
        }
    }

    /**
     * @return {@link SmartOpponent} for chaining.
     */
    public SmartOpponent withJoystick(CommandXboxController controller) {
        this.joystick = Optional.of(controller);
        behaviorChooser.ifPresent(
                chooser -> chooser.addOption("Joystick Drive", Commands.runOnce(() -> setState(States.JOYSTICK))));
        return this;
    }

    /**
     *
     */
    public Command joystickDrive() {
        if (simulation.isPresent() && startPose.isPresent() && currentState.isPresent() && joystick.isPresent()
                && joystick.get() instanceof CommandXboxController controller) {
            return Commands.runOnce(() -> simulation.get().setSimulationWorldPose(startPose.get()))
                    .andThen(Commands.runOnce(() -> simulation.get().runChassisSpeeds(
                            new ChassisSpeeds(), new Translation2d(), false, false), this))
                    .ignoringDisable(true)
                    .andThen(Commands.waitSeconds(1))
                    .andThen(Commands.run(() -> {
                        // Calculate field-centric speed from driverstation speed
                        final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                new ChassisSpeeds(
                                        MathUtil.applyDeadband(-controller.getLeftY(), joystickDeadzone) * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                                        MathUtil.applyDeadband(-controller.getLeftX(), joystickDeadzone) * simulation.get().maxLinearVelocity().in(MetersPerSecond),
                                        MathUtil.applyDeadband(-controller.getRightX(), joystickDeadzone) * simulation.get().maxAngularVelocity().in(RadiansPerSecond)),
                                DriverStation.getAlliance().equals(alliance) ?
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing() :
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                                .plus(Rotation2d.k180deg));
                        // Run the field-centric speed
                        simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false);
                    }, this));
        } else {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }
    }

    /**
     * This should probably just cycle between 2 poses
     *
     * @return
     */
    public Command defendCommand() {
        return Commands.runOnce(() -> DriverStation.reportWarning("No defend state implemented", false), this);
    }

    /**
     * This uses PP NavGrid
     *
     * @param finalPose
     * @return
     */
    public Command pathfindCommand(Pose2d finalPose, Distance tolerance) {
        if (simulation.isPresent() && driveController.isPresent() && mapleADStar.isPresent()) {
            mapleADStar.get().setStartPosition(simulation.get().getActualPoseInSimulationWorld().getTranslation());
            mapleADStar.get().setGoalPosition(finalPose.getTranslation());
            mapleADStar.get().runThread();
            return (Commands.run(() -> {
                        Pose2d currentPose = simulation.get().getActualPoseInSimulationWorld();
                        List<Waypoint> waypoints = mapleADStar.get().currentWaypoints;
                        Translation2d targetTranslation;
                        Rotation2d targetRotation;
                        if (!waypoints.isEmpty()) {
                            targetTranslation = waypoints.get(0).anchor();
                            targetRotation = currentPose.getRotation().interpolate(finalPose.getRotation(), waypoints.size());
                            if (currentPose.getTranslation().getDistance(targetTranslation) < tolerance.in(Meters)) {
                                waypoints.remove(0);
                            }
                        } else {
                            targetTranslation = finalPose.getTranslation();
                            targetRotation = finalPose.getRotation();
                        }
                        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);
                        PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
                        state.pose = targetPose;

                        ChassisSpeeds speeds = driveController.get().calculateRobotRelativeSpeeds(
                                currentPose,
                                state);
                        simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false);
                    }, this)
                    .until(() -> {
                        List<Waypoint> waypoints = mapleADStar.get().currentWaypoints;
                        return waypoints.isEmpty() && nearPose(finalPose, tolerance);
                    })
                    .finallyDo(() -> simulation.get().runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false)));
        } else {
            return Commands.runOnce(() -> {
                DriverStation.reportWarning("No simulation found", false);
            }, this);
        }
    }

    public Pose2d getPose() {
        Pose2d pose = new Pose2d();
        if (simulation.isPresent()) {
            pose = simulation.get().getActualPoseInSimulationWorld();
        }
        return pose;
    }

    /**
     * returns false if simulation isn't present
     */
    public boolean nearPose(Pose2d pose, Distance maxDistance) {
        if (simulation.isPresent()) {
            Translation2d robotTranslation = simulation.get().getActualPoseInSimulationWorld().getTranslation();
            Translation2d goalTranslation = pose.getTranslation();
            double distance = robotTranslation.getDistance(goalTranslation);
            return distance <= maxDistance.in(Meters);
        } else {
            return false;
        }
    }

    /**
     *
     */
    public Command coralFeedShotCommand() {
        // Setup to match the 2025 kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(0.85);
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
     *
     */
    public void coralFeedShot() {
        // Setup to match the 2025 kitbot
        // Coral Settings
        Distance shootHeight = Meters.of(0.85);
        LinearVelocity shootSpeed = MetersPerSecond.of(1);
        Angle shootAngle = Degrees.of(-15);
        Translation2d shootOnBotPosition = new Translation2d(
                0.5,
                0);

        Commands.runOnce(() -> {
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
        }, this);
    }

    /**
     *
     * @return
     */
    public Command algaeFeedShotCommand() {
        // Algae settings
        Distance shootHeight = Meters.of(3);
        LinearVelocity shootSpeed = MetersPerSecond.of(.9);
        Angle shootAngle = Degrees.of(35);
        Translation2d shootOnBotPosition = new Translation2d(
                Inches.of(6).in(Meters),
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
     *
     * @return
     */
    public void algaeFeedShot() {
        // Algae settings
        Distance shootHeight = Meters.of(3);
        LinearVelocity shootSpeed = MetersPerSecond.of(.9);
        Angle shootAngle = Degrees.of(35);
        Translation2d shootOnBotPosition = new Translation2d(
                Inches.of(6).in(Meters),
                0);

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
    }

    /**
     * @return
     */
    public Pose2d getCollectPose() {
        int station = ((int) Math.round(Math.random() * 5));
        boolean willCollide = OpponentManager.getCollectPoses(alliance.get()).stream().anyMatch(Predicate.isEqual(station));
        if (willCollide) {
            station = ((int) Math.round(Math.random() * 5));
        }
        OpponentManager.setCollectPose(id.get(), alliance.get(), station);
        Pose2d stationPose = switch (station) {
            case 0 -> LEFT_STATION_POSE;
            case 1 -> LEFT_STATION_POSE.plus(SLOT_OFFSET_LEFT);
            case 2 -> LEFT_STATION_POSE.plus(SLOT_OFFSET_RIGHT);
            case 3 -> RIGHT_STATION_POSE;
            case 4 -> RIGHT_STATION_POSE.plus(SLOT_OFFSET_LEFT);
            case 5 -> RIGHT_STATION_POSE.plus(SLOT_OFFSET_RIGHT);
            default -> Pose2d.kZero;
        };
        return ifShouldFlip(stationPose);
    }

    /**
     * @return
     */
    public Pose2d getScorePose() {
        this.scoreTarget = Optional.of(((int) Math.round(Math.random() * 17)));
        boolean willCollide = OpponentManager.getTargetPoses(alliance.get()).stream().anyMatch(Predicate.isEqual(scoreTarget.get()));
        if (willCollide) {
            this.scoreTarget = Optional.of(((int) Math.round(Math.random() * 17)));
        }
        OpponentManager.setTargetPose(id.get(), alliance.get(), scoreTarget.get());
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
            case 13, 14, 15, 16, 17 -> BARGE_NET_POSE;
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
