package frc.robot.utils.maplesim.opponents;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class OpponentManager {

    protected static List<Integer> blueTargetPoses = new ArrayList<>();
    protected static List<Integer> redTargetPoses = new ArrayList<>();

    protected static List<Integer> blueCollectPoses = new ArrayList<>();
    protected static List<Integer> redCollectPoses = new ArrayList<>();

    public static void addInitialPoses() {
        for (int i = 0; i < 6; i++) {
            blueTargetPoses.add(100);
            redTargetPoses.add(100);
            blueCollectPoses.add(100);
            redCollectPoses.add(100);
        }
    }

    public static void setTargetPose(int id, DriverStation.Alliance alliance, int selectedTarget) {
        if (alliance == DriverStation.Alliance.Red) {
            redTargetPoses.set(id, selectedTarget);
        } else {
            blueTargetPoses.set(id, selectedTarget);
        }
    }

    public static List<Integer> getTargetPoses(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            return redTargetPoses;
        } else {
            return blueTargetPoses;
        }
    }

    public static void setCollectPose(int id, DriverStation.Alliance alliance, int selectedCollect) {
        if (alliance == DriverStation.Alliance.Red) {
            redCollectPoses.set(id, selectedCollect);
        } else {
            blueCollectPoses.set(id, selectedCollect);
        }
    }

    public static List<Integer> getCollectPoses(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            return redCollectPoses;
        } else {
            return blueCollectPoses;
        }
    }
}
