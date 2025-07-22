package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    // Hardware interface
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;
    private final Drive drive;

    // Motor connection alerts
    private final Alert leaderMotorAlert = new Alert("Elevator leader motor isn't connected", AlertType.kError);
    private final Alert followerMotorAlert = new Alert("Elevator follower motor isn't connected", AlertType.kError);
    private final Alert encoderAlert = new Alert("Elevator encoder isn't connected", AlertType.kError);

    // Mini superstructure state
    private ElevatorMode desiredMode = ElevatorMode.INTAKE;
    private Distance targetDistance = ElevatorMode.INTAKE.targetDistance;

    // Side detection + scoring level tracking
    private String side = "GH";
    private int curentelevator = 4;

    public Elevator(ElevatorIO io, Drive drive) {
        this.io = io;
        this.inputs = new ElevatorIOInputsAutoLogged();
        this.drive = drive;
    }

    @Override
    public void periodic() {
        // Update side from SmartDashboard
        side = SmartDashboard.getString("SelectedRegion", "GH");

        // Update elevator position tracker

        // Update hardware inputs and log
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Motor and encoder health alerts
        leaderMotorAlert.set(!inputs.leaderConnected);
        followerMotorAlert.set(!inputs.followerConnected);
        encoderAlert.set(!inputs.encoderConnected);

        // Superstructure logic
        if (desiredMode == ElevatorMode.STOP) {
            stop();
        } else {
            setDistance(targetDistance);
        }

        // Dashboard
        SmartDashboard.putString("Elevator Mode", desiredMode.name());
        SmartDashboard.putBoolean("Elevator At Target", isAtTarget());
        SmartDashboard.putNumber("Elevator Position (in)", getPosition().in(Inches));
    }

    /** Set the desired elevator mode */
    public void setState(ElevatorMode mode) {
        if (desiredMode != mode) {
            desiredMode = mode;
            targetDistance = mode.targetDistance;
        }
    }

    /** Stop the elevator motors */
    private void stop() {
        io.stop();
    }

    /** Command the elevator to move to a specific distance */
    private void setDistance(Distance distance) {
        io.setDistance(distance);
    }

    /** Get the current elevator position */
    @AutoLogOutput
    public Distance getPosition() {
        return inputs.elevatorDistance;
    }

    /** Return true if elevator is at target */
    @AutoLogOutput
    public boolean isAtTarget() {
        if (desiredMode == ElevatorMode.STOP) return true;
        return getPosition().isNear(desiredMode.targetDistance, desiredMode.distanceTolerance);
    }

    /** Return the target distance */
    @AutoLogOutput
    public Distance getTargetDistance() {
        return desiredMode.targetDistance;
    }

    /** Return which side is currently selected */
    public String whichside() {
        return side;
    }

    /** Increase elevator level */
    public void elevatorup() {
        curentelevator = curentelevator + 1;
    }

    /** Decrease elevator level */
    public void elevatordown() {
        curentelevator = curentelevator - 1;
    }

    /** Return current logical elevator level (1–4) */
    public int elevatorpos() {
        if (curentelevator > 4) {
            curentelevator = 4;
        }
        if (curentelevator < 1) {
            curentelevator = 1;
        }
        return curentelevator;
    }

    /** Elevator logical modes (internal goal states) */
    public enum ElevatorMode {
        STOP(Inches.of(0)),
        Home(Inches.of(0)),
        INTAKE(Inches.of(0)),
        L1(Inches.of(12)),
        L2(Inches.of(24)),
        L3(Inches.of(36)),
        L4(Inches.of(48)),
        BARGE(Inches.of(30)),
        PROCESSOR(Inches.of(20));

        public final Distance targetDistance;
        public final Distance distanceTolerance;

        ElevatorMode(Distance targetDistance) {
            this.targetDistance = targetDistance;
            this.distanceTolerance = Inches.of(2); // Default 2-inch tolerance
        }
    }
}
