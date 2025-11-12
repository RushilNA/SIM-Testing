package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel subsystem implemented as a state machine that mirrors the Intake pattern: - wantedState (requested) -
 * curentState (applied) States: OFF, COLLECT_CORAL, OUTTAKE_L1_HIGH, OUTTAKE_L1_LOW, HOLDING
 *
 * <p>Each state maps to a flywheel velocity setpoint (closed-loop).
 */
public class Flywheel extends SubsystemBase {
    // ───────── IO & inputs ─────────
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;
    private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private final GenericEntry manualBeamBreakEntry = Shuffleboard.getTab("Superstructure")
            .add("Manual Beam Break", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

    // ───────── Alerts ─────────
    private final Alert leaderMotorAlert = new Alert("Flywheel leader motor isn't connected", AlertType.kError);

    // ───────── State enums (match Intake naming) ─────────
    public enum wantedState {
        OFF,
        COLLECT_CORAL,
        OUTTAKE_L1_HIGH,
        OUTTAKE_L1_LOW,
        HOLDING
    }

    public enum curentState {
        OFF,
        COLLECT_CORAL,
        OUTTAKE_L1_HIGH,
        OUTTAKE_L1_LOW,
        HOLDING
    }

    // ───────── Config: target speeds (edit to taste) ─────────
    private static final AngularVelocity kOff = RotationsPerSecond.of(0);
    private static final AngularVelocity kCollect = RotationsPerSecond.of(5); // gentle spin-up
    private static final AngularVelocity kOutL1High = RotationsPerSecond.of(20); // fast shot
    private static final AngularVelocity kOutL1Low = RotationsPerSecond.of(10); // slower shot
    private static final AngularVelocity kHold = RotationsPerSecond.of(0); // anti-stall/hold

    private static final AngularVelocity kTol = RotationsPerSecond.of(1);

    // Track the last non-zero target so HOLDING can “remember” if you prefer
    private AngularVelocity lastNonZeroTarget = kOutL1Low;

    // ───────── State vars ─────────
    private wantedState wState = wantedState.OFF;
    private curentState cState = curentState.OFF;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        this.inputs = new FlywheelIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        leaderMotorAlert.set(!inputs.leaderConnected);

        // Transition: for flywheel we directly realize wanted -> current (no sensor gating like
        // beam-break)
        cState = handleStateTransition();

        // Apply outputs based on current state
        applyState();

        // Logging (match your Intake-style keys)
        Logger.recordOutput("Subsystem/Flywheel/SystemState", cState);
        Logger.recordOutput("Subsystem/Flywheel/WantedState", wState);

        Logger.recordOutput("States/IntakeWheelsSystemState", cState);
        Logger.recordOutput("States/IntakeWheelsWantedState", wState);

        Logger.recordOutput("Subsystem/Flywheel/VelocityRPS", getVelocity().in(RotationsPerSecond));
        Logger.recordOutput(
                "Subsystem/Flywheel/TargetRPS", targetSpeedFor(cState).in(RotationsPerSecond));
        Logger.recordOutput("Subsystem/Flywheel/AtTarget", isAtTarget());
        SmartDashboard.putBoolean("Subsystem/Flywheel/Hascoral", hasCoral());
    }

    // ───────── State logic ─────────

    public boolean hasCoral() {

        return manualBeamBreakEntry.getBoolean(false);
    }

    private curentState handleStateTransition() {
        // If you want special logic (like Intake’s HOLDING vs COLLECT based on a sensor),
        // put it here. For now, we mirror wanted->current directly.
        return switch (wState) {
            case OFF -> curentState.OFF;
            case COLLECT_CORAL -> curentState.COLLECT_CORAL;
            case OUTTAKE_L1_HIGH -> curentState.OUTTAKE_L1_HIGH;
            case OUTTAKE_L1_LOW -> curentState.OUTTAKE_L1_LOW;
            case HOLDING -> {
                if (hasCoral()) {
                    yield curentState.HOLDING;
                } else {
                    yield curentState.COLLECT_CORAL;
                }
            }
        };
    }

    private void applyState() {
        AngularVelocity tgt = targetSpeedFor(cState);
        if (tgt.gt(kOff)) {
            lastNonZeroTarget = tgt;
            io.setVelocity(tgt);
        } else {
            io.stop();
        }
    }

    private AngularVelocity targetSpeedFor(curentState s) {
        return switch (s) {
            case OFF -> kOff;
            case COLLECT_CORAL -> kCollect;
            case OUTTAKE_L1_HIGH -> kOutL1High;
            case OUTTAKE_L1_LOW -> kOutL1Low;
            case HOLDING -> kHold; // or lastNonZeroTarget if you prefer: lastNonZeroTarget
        };
    }

    // ───────── Public API (same feel as Intake) ─────────

    public void setWantedState(wantedState state) {
        this.wState = state;
    }

    public void stop() {
        setWantedState(wantedState.OFF);
    }

    /** Convenience “actions” matching Intake-style names if you want parity. */
    public void intakeIn() { // maps to gentle spin
        setWantedState(wantedState.COLLECT_CORAL);
    }

    public void intakeOutSlow() { // slower shot
        setWantedState(wantedState.OUTTAKE_L1_LOW);
    }

    public void collectCoral() { // hold
        setWantedState(wantedState.HOLDING);
    }

    // Command wrapper (parallel to your Intake’s setStateCommand(double))
    public Command setStateCommand(wantedState state) {
        return new Command() {
            @Override
            public void initialize() {
                setWantedState(state);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    // ───────── Telemetry helpers ─────────

    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return inputs.leaderVelocity;
    }

    @AutoLogOutput
    public boolean isAtTarget() {
        AngularVelocity tgt = targetSpeedFor(cState);
        if (tgt.equals(kOff)) return true;
        return getVelocity().isNear(tgt, kTol);
    }
}
