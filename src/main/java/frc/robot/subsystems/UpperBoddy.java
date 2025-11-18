package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

/**
 * Upper-body coordinator that runs a state machine over Elevator + Arm. - Mirrors the earlier pattern: WantedState /
 * SystemState + handle/apply. - Uses Arm/Elevator public APIs (no direct IO). - Includes safe path planning (lift then
 * swing if needed).
 */
public class UpperBoddy extends SubsystemBase {

    /** Targets / limits (tune for your robot) */
    public static final Distance kElevatorMin = Meters.of(0.00);

    public static final Distance kElevatorMax = Meters.of(1.30);
    public static final Angle kShoulderMin = Degrees.of(-10.0);
    public static final Angle kShoulderMax = Degrees.of(270.0);

    /** If elevator below this, shoulder must avoid the “wide swing” unsafe band. */
    public static final Distance kSafeElevatorForWideAngles = Meters.of(0.35);

    public static final Angle kUnsafeLowAngleMin = Degrees.of(-5.0);
    public static final Angle kUnsafeLowAngleMax = Degrees.of(70.0);

    /** Setpoint tolerances used by reachedSetpoint() */
    public static final Distance kElevatorTol = Centimeters.of(1.0); // ~1 cm

    public static final Angle kShoulderTol = Degrees.of(1.5);

    /** Simple pose container */
    public record Pose(Distance elev, Angle shoulder) {}

    /** External requests */
    public enum WantedState {
        /** Run homing (go to intake/home pose), then idle-hold. */
        HOME,
        /** Hold current pose; don’t change targetPose. */
        IDLE,
        /** Move to targetPose (set by setTargetPose). */
        MOVE_TO_POSITION
    }

    /** Internal execution modes */
    private enum SystemState {
        HOMING,
        IDLING,
        MOVING_TO_TARGET
    }

    private final Elevator elevator;
    private final Arm arm;

    // State
    private WantedState wanted = WantedState.HOME;
    private SystemState system = SystemState.HOMING;
    private boolean initialHomeComplete = false;

    // Target pose (default home)
    private Pose targetPose = new Pose(Meters.of(0.0), Degrees.of(180));

    public UpperBoddy(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    // ───────────────────────────────── periodic ─────────────────────────────────
    @Override
    public void periodic() {
        // Decide
        system = handleStateTransitions();

        // Act
        applyState(); // make sure i add this part back after testing

        // Log
        Logger.recordOutput("Superstructure/SystemState", system.name());
        Logger.recordOutput("Superstructure/WantedState", wanted.name());
        Logger.recordOutput("Superstructure/TargetElev_m", targetPose.elev.in(Meters));
        Logger.recordOutput("Superstructure/TargetShoulder_deg", targetPose.shoulder.in(Degrees));
        Logger.recordOutput("Superstructure/AtSetpoint", reachedSetpoint());
        Logger.recordOutput(
                "Superstructure/ElevatorPos_m", elevator.getPosition().in(Meters));
        Logger.recordOutput("Superstructure/ArmPos_deg", arm.getPosition().in(Degrees));
    }

    // ────────────────────────────── Public API ────────────────────────────────
    /** Request a state (HOME / IDLE / MOVE_TO_POSITION). */
    public void setWantedState(WantedState state) {
        this.wanted = state;
    }

    /** Set desired pose and request MOVE_TO_POSITION. */
    public void setTargetPose(Pose pose) {
        this.targetPose = clamp(pose);
        this.wanted = WantedState.MOVE_TO_POSITION;
    }

    /** Capture current measured pose and hold there (IDLE). */
    public void holdCurrentAsIdle() {
        this.targetPose = new Pose(elevator.getPosition(), arm.getPosition());
        this.wanted = WantedState.IDLE;
    }

    /** Convenience: request HOME. */
    public void requestHome() {
        this.wanted = WantedState.HOME;
    }

    /** Check both axes near their targets (unit-safe). */
    public boolean reachedSetpoint() {
        return elevator.getPosition().isNear(targetPose.elev, kElevatorTol)
                && arm.getPosition().isNear(targetPose.shoulder, kShoulderTol);
    }

    public double getelevatorpos() {
        return elevator.getPosition().in(Meters);
    }

    public double getarm() {
        return arm.getPosition().in(Degrees);
    }

    public void armfirst() {}

    // ─────────────────────────── Transition function ───────────────────────────
    private SystemState handleStateTransitions() {
        if (DriverStation.isEStopped() || DriverStation.isDisabled()) {
            return SystemState.IDLING;
        }

        return switch (wanted) {
            case HOME -> SystemState.HOMING;
            case IDLE -> {
                if (!initialHomeComplete) yield SystemState.HOMING;
                yield SystemState.IDLING;
            }
            case MOVE_TO_POSITION -> {
                if (!initialHomeComplete) yield SystemState.HOMING;
                yield SystemState.MOVING_TO_TARGET;
            }
        };
    }

    // ────────────────────────────── Apply function ─────────────────────────────
    private void applyState() {
        switch (system) {
            case HOMING -> {
                // Define your “home/intake” pose
                Pose home = new Pose(Meters.of(0.0), Degrees.of(180));

                // Go there (simple, safe)
                commandToPose(home);

                // Consider homed once we’re near
                if (elevator.getPosition().isNear(home.elev, kElevatorTol)
                        && arm.getPosition().isNear(home.shoulder, kShoulderTol)) {
                    initialHomeComplete = true;
                    targetPose = home;
                }
            }

            case IDLING -> {
                // Hold current targets (no change)
                commandToPose(targetPose);
            }

            case MOVING_TO_TARGET -> {
                // Safe planner: lift first if needed, then swing
                Pose planned = planSafePath(targetPose);
                commandToPose(planned);
            }
        }
    }

    // ───────────────────────────── Helper functions ────────────────────────────
    /** Clamp pose to mech limits. */
    private static Pose clamp(Pose p) {
        double em = MathUtil.clamp(p.elev.in(Meters), kElevatorMin.in(Meters), kElevatorMax.in(Meters));
        double sd = MathUtil.clamp(p.shoulder.in(Degrees), kShoulderMin.in(Degrees), kShoulderMax.in(Degrees));
        return new Pose(Meters.of(em), Degrees.of(sd));
    }

    private static boolean isInUnsafeLowBand(Angle a) {
        double d = a.in(Degrees);
        return d >= kUnsafeLowAngleMin.in(Degrees) && d <= kUnsafeLowAngleMax.in(Degrees);
    }

    private static Angle clampToSafeBand(Angle current) {
        if (!isInUnsafeLowBand(current)) return current;
        double d = current.in(Degrees);
        double dToMin = Math.abs(d - kUnsafeLowAngleMin.in(Degrees));
        double dToMax = Math.abs(d - kUnsafeLowAngleMax.in(Degrees));
        return (dToMin < dToMax) ? kUnsafeLowAngleMin : kUnsafeLowAngleMax;
    }

    /** Current measured pose (from sensors/estimates on Elevator + Arm). */
    public Pose getMeasuredPose() {
        return new Pose(elevator.getPosition(), arm.getPosition());
    }

    public double getpos() {
        return arm.getPosition().in(Degrees);
    }

    /** Current commanded/goal pose (what the state machine is driving toward). */
    public Pose getTargetPose() {
        return targetPose;
    }

    /** Safe path: if elev low AND desired shoulder in unsafe band, lift first while holding shoulder safe. */
    private Pose planSafePath(Pose desired) {
        var elevNow = elevator.getPosition();
        var shNow = arm.getPosition();

        boolean elevTooLow = elevNow.lt(kSafeElevatorForWideAngles);
        boolean desiredUnsafe = isInUnsafeLowBand(desired.shoulder);

        if (elevTooLow && desiredUnsafe) {
            double stagedMeters = Math.max(desired.elev.in(Meters), kSafeElevatorForWideAngles.in(Meters));
            Angle stagedShoulder = clampToSafeBand(shNow);
            return clamp(new Pose(Meters.of(stagedMeters), stagedShoulder));
        }
        return clamp(desired);
    }

    /** Push setpoints into subsystems (closed-loop inside each). */
    private void commandToPose(Pose pose) {
        elevator.setDistance(pose.elev); // requires tiny helper added below
        arm.setAngle(pose.shoulder); // requires tiny helper added below
    }
}
