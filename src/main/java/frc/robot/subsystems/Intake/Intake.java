package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
    public double targetAngle = 0.0;

    // target setpoint in degrees
    private double angleDeg = 0.0;

    public enum wantedState {
        OFF,
        COLLECT_CORAL,
        OUTTAKE_L1_HIGH,
        OUTTAKE_L1_LOW,
        HOLDING,
        HAND_OFF,
        HOME,
        Moving
    }

    public enum curentState {
        OFF,
        COLLECT_CORAL,
        OUTTAKE_L1_HIGH,
        OUTTAKE_L1_LOW,
        HOLDING,
        HAND_OFF,
        HOME,
        Moving
    }

    private wantedState wState = wantedState.OFF;
    private curentState cState = curentState.OFF;

    private final Alert leaderMotorAlert = new Alert("Intake pivot leader motor isn't connected", AlertType.kError);

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Pivot", inputs);

        leaderMotorAlert.set(!inputs.motorConnected);

        Logger.recordOutput("Intake1/PIV/SystemState", cState);
        Logger.recordOutput("Intake1/PIV/WantedState", wState);

        Logger.recordOutput("States/IntakeSystemState", cState);
        Logger.recordOutput("States/IntakeWantedState", wState);

        Logger.recordOutput("Intake1/PIV/Position_deg", getPosition().in(Degrees));
        Logger.recordOutput("Intake1/PIV/Target_deg", angleDeg);

        cState = handleStateTransition();
        applyState();
    }

    /** Current measured angle from inputs (ensure your IO populates this). */
    @AutoLogOutput
    public Angle getPosition() {
        return inputs.intakeAngle; // must be an Angle measure
    }

    public void setTargetAngleDeg(double deg) {
        angleDeg = deg;
        io.setPosition(Degrees.of(deg)); // pass an Angle measure in DEGREES
    }

    private curentState handleStateTransition() {
        return switch (wState) {
            case HOLDING -> curentState.HOLDING;
            case OUTTAKE_L1_HIGH -> curentState.OUTTAKE_L1_HIGH;
            case OUTTAKE_L1_LOW -> curentState.OUTTAKE_L1_LOW;
            case OFF -> curentState.OFF;
            case COLLECT_CORAL -> curentState.COLLECT_CORAL;
            case HAND_OFF -> curentState.HAND_OFF;
            case HOME -> curentState.HOME;
            case Moving -> curentState.Moving; // if you intend to use it
        };
    }

    public void applyState() {
        switch (cState) {
            case HOME:
                targetAngle = 0;
                break;
            case COLLECT_CORAL:
                targetAngle = 90;
                break;
            case OUTTAKE_L1_HIGH:
                targetAngle = 45;
                break;
            case OUTTAKE_L1_LOW:
                targetAngle = 45;
                break;
            case HOLDING:
                targetAngle = 0;
                break;
            case HAND_OFF:
                targetAngle = 35;
                break;
            case OFF:
                targetAngle = 0;
                break;
            case Moving:
                break;

            default:
                break;
        }
        setTargetAngleDeg(targetAngle);
    }

    // *** FIXED: make this a real setter that actually changes state immediately
    public void setWantedState(wantedState state) {
        this.wState = state;
    }

    // Convenience commands if you want command-based triggers:
    public Command setWantedStateCommand(wantedState state) {
        return Commands.runOnce(() -> setWantedState(state), this);
    }

    public void Home() {
        setWantedState(wantedState.HOME);
    }

    public void Handoff() {
        setWantedState(wantedState.HAND_OFF);
    }

    // If you really want a one-shot command:
    public Command setCollectCommand() {
        return Commands.runOnce(() -> setWantedState(wantedState.COLLECT_CORAL), this);
    }

    @AutoLogOutput
    public boolean isAtTarget() {
        return getPosition().isNear(Degrees.of(angleDeg), Degrees.of(2)); // 2° tolerance
    }

    @AutoLogOutput
    private Angle targetAngle() {
        return Degrees.of(angleDeg);
    }
}
