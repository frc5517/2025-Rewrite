package frc.robot.utils.maplesim;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.maplesim.opponents.KitBot;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;

import java.util.Arrays;

public class MapleSim extends SubsystemBase {
    private static final Timer matchTimer = new Timer();
    public static IntegerPublisher coralScoredInL1RedPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance")
            .getIntegerTopic("Coral Scored in Trough")
            .publish();
    public static IntegerPublisher coralScoredInL2RedPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance")
            .getIntegerTopic("Coral Scored on L2")
            .publish();
    public static IntegerPublisher coralScoredInL3RedPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance")
            .getIntegerTopic("Coral Scored in L3")
            .publish();
    public static IntegerPublisher coralScoredInL4RedPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance")
            .getIntegerTopic("Coral Scored in L4")
            .publish();
    public static StringPublisher reefRedPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance")
            .getStringTopic("Reef AtoL[L1, L2, L3, L4]")
            .publish();
    public static IntegerPublisher algaeInNetRedPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance")
            .getIntegerTopic("Algae in net")
            .publish();
    public static IntegerPublisher redScorePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData")
            .getIntegerTopic("Red Alliance Score")
            .publish();
    public static IntegerPublisher coralScoredInL1BluePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance")
            .getIntegerTopic("Coral Scored in Trough")
            .publish();
    public static IntegerPublisher coralScoredInL2BluePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance")
            .getIntegerTopic("Coral Scored on L2")
            .publish();
    public static IntegerPublisher coralScoredInL3BluePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance")
            .getIntegerTopic("Coral Scored in L3")
            .publish();
    public static IntegerPublisher coralScoredInL4BluePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance")
            .getIntegerTopic("Coral Scored in L4")
            .publish();
    public static StringPublisher reefBluePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance")
            .getStringTopic("Reef AtoL[L1, L2, L3, L4]")
            .publish();
    public static IntegerPublisher algaeInNetBluePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData/Breakdown/Blue Alliance")
            .getIntegerTopic("Algae in net")
            .publish();
    public static IntegerPublisher blueScorePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData")
            .getIntegerTopic("Blue Alliance Score")
            .publish();
    public static DoublePublisher scoreTimePublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/MatchData")
            .getDoubleTopic("Current Match Time")
            .publish();
    public static long coralOnL1Red = 0;
    public static long coralOnL2Red = 0;
    public static long coralOnL3Red = 0;
    public static long coralOnL4Red = 0;
    public static long algaeScoredInNetBlue = 0;
    public static long coralOnL1Blue = 0;
    public static long coralOnL2Blue = 0;
    public static long coralOnL3Blue = 0;
    public static long coralOnL4Blue = 0;
    public static long algaeScoredInNetRed = 0;
    private final StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/GamePieces")
            .getStructArrayTopic("Coral Array",
                    Pose3d.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim/GamePieces")
            .getStructArrayTopic("Algae Array",
                    Pose3d.struct)
            .publish();
    private ReefscapeReefSimulation reef;

    private MapleSim() {
        setupMatchData();
    }

    /**
     * Add "MapleSim.maplesimInit();" to simulationInit() in Robot.java to start everything.
     * DO NOT create another {@link MapleSim} instance this method creates it for you.
     */
    public static void mapleSimInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
        new MapleSim();
        new KitBot(0, DriverStation.Alliance.Blue)
                .withControls(new CommandXboxController(3));
        new KitBot(1, DriverStation.Alliance.Blue);
        new KitBot(2, DriverStation.Alliance.Blue);
        new KitBot(3, DriverStation.Alliance.Red);
        new KitBot(4, DriverStation.Alliance.Red);
    }

    /**
     * If you don't already schedule the pathplanner warm up command then this should be called after mapleSimInit().
     */
    public static void pathplannerWarmup() {
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * It's recommended to just use your own intake/shoot sim commands.
     * This method runs the given intake with a timeout for {@param seconds} before
     * adding a piece to the intake anyway.
     *
     * @param intake  {@link IntakeSimulation} to use.
     * @param seconds until timeout.
     */
    public static void simIntake(IntakeSimulation intake, double seconds) {
        Commands.run(intake::startIntake)
                .withTimeout(seconds)
                .andThen(intake::stopIntake)
                .andThen(intake::addGamePieceToIntake);
    }

    /**
     * This method just gets {@link SimulatedArena} adds the given game piece to the sim and does some score keeping.
     * Algae is not counted unless this method is used, or you manually add a piece to the score count.
     *
     * @param shotGamePiece {@link GamePieceProjectile} to shoot.
     */
    public static void simAlgaeFeedshot(GamePieceProjectile shotGamePiece) {
        SimulatedArena.getInstance().addGamePieceProjectile(shotGamePiece);
        if (shotGamePiece instanceof ReefscapeAlgaeOnFly) {
            if (shotGamePiece.willHitTarget()) {
                if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    algaeScoredInNetRed++;
                } else {
                    algaeScoredInNetBlue++;
                }
            }
        }
    }

    /**
     * This method just gets {@link SimulatedArena} and adds {@param shotGamePiece} to the sim;
     * but only if there is a game piece in the intake, a game piece is then deducted from the intake.
     *
     * @param shotGamePiece {@link GamePieceProjectile} to shoot.
     * @param intake        {@link IntakeSimulation} to use.
     */
    public static void simShotIfHasPiece(GamePieceProjectile shotGamePiece, IntakeSimulation intake) {
        if (intake.obtainGamePieceFromIntake()) {
            simAlgaeFeedshot(shotGamePiece);
        }
    }

    /**
     * Just another basic method that uses in-built Maple-Sim functions.
     * Drops a coral at all stations at once.
     *
     * @param isHorizontal whether the coral should be horizontal.
     */
    public static void addCoralAllStations(boolean isHorizontal) {
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.LEFT_STATION,
                DriverStation.Alliance.Blue,
                isHorizontal
        ));
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.RIGHT_STATION,
                DriverStation.Alliance.Blue,
                isHorizontal
        ));
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.LEFT_STATION,
                DriverStation.Alliance.Red,
                isHorizontal
        ));
        SimulatedArena.getInstance().addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.RIGHT_STATION,
                DriverStation.Alliance.Red,
                isHorizontal
        ));
    }

    public static void addAlgaeToScore(DriverStation.Alliance alliance, int toScore) {
        if (alliance == DriverStation.Alliance.Red) {
            algaeScoredInNetRed += toScore;
        } else if (alliance == DriverStation.Alliance.Blue) {
            algaeScoredInNetBlue += toScore;
        }
    }

    public static void clearMatchData() {
        matchTimer.restart();
        coralOnL1Red = 0;
        coralOnL2Red = 0;
        coralOnL3Red = 0;
        coralOnL4Red = 0;
        algaeScoredInNetBlue = 0;

        coralOnL1Blue = 0;
        coralOnL2Blue = 0;
        coralOnL3Blue = 0;
        coralOnL4Blue = 0;
        algaeScoredInNetRed = 0;
    }

    private void setupMatchData() {
        if (ReefscapeReefSimulation.getInstance().isPresent()) {
            reef = ReefscapeReefSimulation.getInstance().get();
        }
        matchTimer.start();
        clearMatchData();
    }

    private void updateMatchDataReefscape() {
        coralOnL1Red = getAtoL(DriverStation.Alliance.Red, 0);
        coralOnL2Red = getAtoL(DriverStation.Alliance.Red, 1);
        coralOnL3Red = getAtoL(DriverStation.Alliance.Red, 2);
        coralOnL4Red = getAtoL(DriverStation.Alliance.Red, 3);

        coralOnL1Blue = getAtoL(DriverStation.Alliance.Blue, 0);
        coralOnL2Blue = getAtoL(DriverStation.Alliance.Blue, 1);
        coralOnL3Blue = getAtoL(DriverStation.Alliance.Blue, 2);
        coralOnL4Blue = getAtoL(DriverStation.Alliance.Blue, 3);

        reefBluePublisher.set(Arrays.deepToString(reef.getBranches(DriverStation.Alliance.Blue)));
        reefRedPublisher.set(Arrays.deepToString(reef.getBranches(DriverStation.Alliance.Red)));

        coralScoredInL1RedPublisher.set(coralOnL1Red);
        coralScoredInL2RedPublisher.set(coralOnL2Red);
        coralScoredInL3RedPublisher.set(coralOnL3Red);
        coralScoredInL4RedPublisher.set(coralOnL4Red);

        coralScoredInL1BluePublisher.set(coralOnL1Blue);
        coralScoredInL2BluePublisher.set(coralOnL2Blue);
        coralScoredInL3BluePublisher.set(coralOnL3Blue);
        coralScoredInL4BluePublisher.set(coralOnL4Blue);

        algaeInNetRedPublisher.set(algaeScoredInNetRed);
        algaeInNetBluePublisher.set(algaeScoredInNetBlue);

        long scoreOnL1 = 2;
        long scoreOnL2 = 3;
        long scoreOnL3 = 4;
        long scoreOnL4 = 5;
        long scoreInNet = 4;

        redScorePublisher.set(
                coralOnL1Red * scoreOnL1 +
                        coralOnL2Red * scoreOnL2 +
                        coralOnL3Red * scoreOnL3 +
                        coralOnL4Red * scoreOnL4 +
                        algaeScoredInNetRed * scoreInNet);
        blueScorePublisher.set(
                coralOnL1Blue * scoreOnL1 +
                        coralOnL2Blue * scoreOnL2 +
                        coralOnL3Blue * scoreOnL3 +
                        coralOnL4Blue * scoreOnL4 +
                        algaeScoredInNetBlue * scoreInNet);

        // Rounded to hundredth place.
        scoreTimePublisher.set(
                Math.round(matchTimer.get() * 100) / 100.0);
    }

    private int getAtoL(DriverStation.Alliance alliance, int level) {
        return
                reef.getBranches(alliance)[0][level] +
                        reef.getBranches(alliance)[1][level] +
                        reef.getBranches(alliance)[2][level] +
                        reef.getBranches(alliance)[3][level] +
                        reef.getBranches(alliance)[4][level] +
                        reef.getBranches(alliance)[5][level] +
                        reef.getBranches(alliance)[6][level] +
                        reef.getBranches(alliance)[7][level] +
                        reef.getBranches(alliance)[8][level] +
                        reef.getBranches(alliance)[9][level] +
                        reef.getBranches(alliance)[10][level] +
                        reef.getBranches(alliance)[11][level];
    }

    @Override
    public void simulationPeriodic() {
        updateMatchDataReefscape();
        // Get the positions of the notes (both on the field and in the air);
        coralPoses.set(SimulatedArena.getInstance()
                .getGamePiecesByType("Coral")
                .toArray(Pose3d[]::new)
        );
        algaePoses.set(SimulatedArena.getInstance()
                .getGamePiecesByType("Algae")
                .toArray(Pose3d[]::new)
        );
    }
}

