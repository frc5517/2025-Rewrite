package frc.robot.utils.maplesim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class MapleSim extends SubsystemBase {
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

    /**
     * Add "MapleSim.maplesimInit();" to simulationInit() in Robot.java to start everything.
     * DO NOT create another {@link MapleSim} instance this method creates it for you.
     */
    public static void mapleSimInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
        SimulatedArena.getInstance().placeGamePiecesOnField();
        SimulatedRobots.startOpponentRobotSimulations();
        SimulatedArena.getInstance().resetFieldForAuto();
        new MapleSim();
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
     * It's recommended to just use your own intake/shoot sim commands.
     * This method just gets {@link SimulatedArena} and adds the given game piece to the sim.
     *
     * @param shotGamePiece {@link GamePieceProjectile} to shoot.
     */
    public static void simShot(GamePieceProjectile shotGamePiece) {
        SimulatedArena.getInstance().addGamePieceProjectile(shotGamePiece);
    }

    /**
     * It's recommended to just use your own intake/shoot sim commands.
     * This method just gets {@link SimulatedArena} and adds {@param shotGamePiece} to the sim;
     * but only if there is a game piece in the intake, a game piece is then deducted from the intake.
     *
     * @param shotGamePiece {@link GamePieceProjectile} to shoot.
     * @param intake        {@link IntakeSimulation} to use.
     */
    public static void simShotIfHasPiece(GamePieceProjectile shotGamePiece, IntakeSimulation intake) {
        if (intake.obtainGamePieceFromIntake()) {
            simShot(shotGamePiece);
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

    @Override
    public void simulationPeriodic() {
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

