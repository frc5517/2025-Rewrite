package frc.robot.utils.maplesim.opponents;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class KitBot extends SmartOpponent {

    protected Optional<CommandXboxController> controller = Optional.empty();

    public KitBot(int id, DriverStation.Alliance alliance) {
        this.opponentMassKG = Optional.of(Kilograms.of(55));
        this.opponentMOI = Optional.of(8.0);
        this.opponentWheelRadius = Optional.of(Inches.of(2));
        this.opponentDriveVelocity = Optional.of(MetersPerSecond.of(8.5));
        this.opponentDriveCOF = Optional.of(1.19);
        this.opponentDriveMotor = Optional.of(DCMotor.getNEO(1)
                .withReduction(8.14));
        this.opponentDriveCurrentLimit = Optional.of(40.0);
        this.opponentNumDriveMotors = Optional.of(1);
        this.opponentTrackWidth = Optional.of(Inches.of(23));
        setupOpponent(id, alliance);
    }

    @Override
    public void buildBehaviorChooser() {
        super.buildBehaviorChooser();
        behaviorChooser.ifPresent(
                chooser -> chooser.addOption("Joystick Drive", Commands.runOnce(() -> setState(States.JOYSTICK))));
    }

    public Command joystickState() {
        return super.joystickDrive(
                () -> getController().getLeftY(),
                () -> getController().getLeftX(),
                () -> getController().getRightX());
    }

    /**
     * @return
     */
    public SmartOpponent withControls(CommandXboxController controller) {
        this.controller = Optional.of(controller);
        return this;
    }

    public CommandXboxController getController() {
        if (controller.isPresent()) {
            return controller.get();
        } else {
            DriverStation.reportWarning("Command Xbox Controller not found, use withControls()", false);
            return null;
        }
    }
}
