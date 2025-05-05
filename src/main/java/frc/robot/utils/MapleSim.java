package frc.robot.utils;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.maplesim.SimulatedRobots;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.units.Units.*;

public class MapleSim extends SubsystemBase {
    private IntakeSimulation intakeSimulation = null;
    private final Pose3d intakedNotePose = new Pose3d(
            new Translation3d(
                    0,
                    0,
                    0),
            new Rotation3d(0, 0, 0));

    private final StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim")
            .getStructArrayTopic("Coral Array",
                    Pose3d.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard/MapleSim")
            .getStructArrayTopic("Algae Array",
                    Pose3d.struct)
            .publish();
    private final List<Pair<String, List<Pose3d>>> mechList = new ArrayList<>();

    public MapleSim() {
        for (Pair<String, List<Pose3d>> mech : mechList) {
            String name = mech.getFirst();
            StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
                    .getStructTopic("Mech3D/" + name + "Pose", Pose3d.struct).publish();
            StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Mech3D/" + name + "PoseArray", Pose3d.struct).publish();

        }
    }

    public void addMechPoses(String mechName, Pose3d... poses) {
        List<Pose3d> posesList = new ArrayList<>(Arrays.asList(poses));
        Pair<String, List<Pose3d>> mech = new Pair<>(mechName, posesList);
        mechList.add(mech);
    }

    public static void mapleSimInit() {
        SimulatedRobots.startOpponentRobotSimulations();
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void mapleSimIntakeSetup(SwerveSubsystem swerveSubsystem)
    {
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                "Coral",
                swerveSubsystem.getMapleDrive(),
                Inches.of(5),
                IntakeSimulation.IntakeSide.FRONT,
                1
        );
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

        for (Pair<String, List<Pose3d>> mech : mechList) {

        }

    }
}

