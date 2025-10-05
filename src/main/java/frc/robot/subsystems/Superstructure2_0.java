// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Inches;

// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Intake.Intake;
// import frc.robot.subsystems.Intake.Intake.IntakeMode;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.Arm.ArmMode;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.Elevator.ElevatorMode;
// import java.util.EnumMap;
// import java.util.Objects;

// /**
//  * Superstructure2_0
//  *
//  * <p>Coordinator for Elevator + Arm + Intake. - Minimal transitions (no auto-home). - Safety nets ONLY in STOW:
// before
//  * moving the arm to stow, auto-raise elevator to a clearance height. - Parallel Elevator+Arm when safe (outside
// STOW).
//  * - After any temporary lift (during STOW), elevator returns to the requested final target.
//  */
// public class Superstructure2_0 extends SubsystemBase {

//     /** Robot-level states requested by driver code. */
//     public enum State {
//         L1,
//         L2,
//         L3,
//         L4,
//         BARGE,
//         INTAKE,
//         INTAKE_SOURCE, // add IntakeMode.SOURCE later if you want a distinct angle
//         ALGAE_POPSICLE,
//         ALGAE_L1,
//         ALGAE_L2,
//         PROCESSOR
//     }

//     /** Per-state target modes for each subsystem. */
//     private static final class Goal {
//         final ElevatorMode elevator;
//         final ArmMode arm;
//         final IntakeMode intake;

//         Goal(ElevatorMode e, ArmMode a, IntakeMode i) {
//             this.elevator = e;
//             this.arm = a;
//             this.intake = i;
//         }
//     }

//     // ------------------ Safety policy (used ONLY during STOW) ------------------

//     /** When must we stow first? (From->To pairs that are risky) */
//     private static final class SafetyRules {
//         static boolean requireStow(State from, State to) {
//             // Example: going to very tall/awkward states from low intake stance
//             return (to == State.L4 || to == State.BARGE) && (from == State.INTAKE || from == State.L1);
//         }

//         static boolean allowParallelAE(State to) {
//             // Allow elevator+arm parallel moves except at the riskiest spots
//             return to != State.L4 && to != State.BARGE;
//         }
//     }

//     /** Minimum elevator height (inches) required BEFORE moving arm to STOW (ArmMode.INTAKE). */
//     private static double minElevatorInchesForStowArm() {
//         // Arm stow ~0°: keep elevator up to avoid pinch/crush
//         return 10.0;
//     }

//     // ------------------ End goals per high-level State ------------------

//     private static final EnumMap<State, Goal> GOALS = new EnumMap<>(State.class);

//     static {
//         // NOTE: Intake currently exposes STOP, Home, INTAKE (case-sensitive!). Using Home as "stow/safe".
//         GOALS.put(State.INTAKE, new Goal(ElevatorMode.INTAKE, ArmMode.INTAKE, IntakeMode.INTAKE));
//         GOALS.put(State.L1, new Goal(ElevatorMode.L1, ArmMode.L1, IntakeMode.Home));
//         GOALS.put(State.L2, new Goal(ElevatorMode.L2, ArmMode.L2, IntakeMode.Home));
//         GOALS.put(State.L3, new Goal(ElevatorMode.L3, ArmMode.L3, IntakeMode.Home));
//         GOALS.put(State.L4, new Goal(ElevatorMode.L4, ArmMode.L4, IntakeMode.Home));
//         GOALS.put(State.BARGE, new Goal(ElevatorMode.BARGE, ArmMode.L2, IntakeMode.Home)); // tweak Arm as needed
//         GOALS.put(State.PROCESSOR, new Goal(ElevatorMode.PROCESSOR, ArmMode.L1, IntakeMode.Home));

//         // Placeholders until you add explicit arm/intake modes for these
//         GOALS.put(
//                 State.INTAKE_SOURCE,
//                 new Goal(ElevatorMode.INTAKE, ArmMode.INTAKE, IntakeMode.INTAKE /* SOURCE later */));
//         GOALS.put(State.ALGAE_POPSICLE, new Goal(ElevatorMode.L2, ArmMode.L2, IntakeMode.Home));
//         GOALS.put(State.ALGAE_L1, new Goal(ElevatorMode.L1, ArmMode.L1, IntakeMode.Home));
//         GOALS.put(State.ALGAE_L2, new Goal(ElevatorMode.L2, ArmMode.L2, IntakeMode.Home));
//     }

//     // ------------------ Engine ------------------

//     private enum Phase {
//         IDLE,
//         STOW, // apply safety nets here (clearance before arm stow)
//         AE_PARALLEL, // move Arm + Elevator together (fast path, no extra safety bumps)
//         MOVE_ELEVATOR,
//         MOVE_ARM,
//         MOVE_INTAKE,
//         MOVE_ELEVATOR_TO_FINAL, // return elevator to final target after a temporary lift in STOW
//         HOLD
//     }

//     private final Elevator elevator;
//     private final Arm arm;
//     private final Intake intake;

//     private State currentState = State.INTAKE;
//     private State requestedState = State.INTAKE;

//     private Goal currentGoal = GOALS.get(State.INTAKE); // what we're presently "holding"
//     private Phase phase = Phase.HOLD;

//     private final Timer timer = new Timer();

//     public Superstructure2_0(Elevator elevator, Arm arm, Intake intake) {
//         this.elevator = Objects.requireNonNull(elevator);
//         this.arm = Objects.requireNonNull(arm);
//         this.intake = Objects.requireNonNull(intake);
//         timer.start();
//     }

//     /** Request a new high-level state. Non-blocking; transition planned on the fly. */
//     public void requestState(State newState) {
//         if (newState == null || newState == requestedState) return;

//         requestedState = newState;
//         final boolean needStow = SafetyRules.requireStow(currentState, requestedState);

//         if (needStow) {
//             phase = Phase.STOW; // retract to safe posture first (with safety nets)
//         } else if (SafetyRules.allowParallelAE(requestedState)) {
//             phase = Phase.AE_PARALLEL; // move elevator+arm together when both need changes
//         } else {
//             // Choose the first thing that actually changed
//             Goal to = GOALS.get(requestedState);
//             boolean eDiff = currentGoal.elevator != to.elevator;
//             boolean aDiff = currentGoal.arm != to.arm;
//             boolean iDiff = currentGoal.intake != to.intake;

//             if (eDiff) phase = Phase.MOVE_ELEVATOR;
//             else if (aDiff) phase = Phase.MOVE_ARM;
//             else if (iDiff) phase = Phase.MOVE_INTAKE;
//             else phase = Phase.HOLD; // already there
//         }
//         timer.reset();
//     }

//     public State getRequestedState() {
//         return requestedState;
//     }

//     public State getCurrentState() {
//         return currentState;
//     }

//     public boolean isBusy() {
//         return phase != Phase.IDLE && phase != Phase.HOLD;
//     }

//     @Override
//     public void periodic() {
//         final Goal to = GOALS.get(requestedState);
//         if (to == null) return;

//         final boolean eDiff = currentGoal.elevator != to.elevator;
//         final boolean aDiff = currentGoal.arm != to.arm;
//         final boolean iDiff = currentGoal.intake != to.intake;

//         switch (phase) {
//             case STOW: {
//                 // SAFETY NETS LIVE HERE:
//                 // Before pulling arm to stow (ArmMode.INTAKE) ensure elevator is above clearance
//                 final double needIn = minElevatorInchesForStowArm();
//                 final double haveIn = elevator.getPosition().in(Inches);

//                 if (haveIn + 0.05 < needIn) {
//                     // Raise to a discrete elevator mode that meets/exceeds clearance
//                     ElevatorMode clearanceMode = pickElevatorModeAtLeast(needIn);
//                     if (clearanceMode == null) clearanceMode = ElevatorMode.L4; // conservative fallback
//                     elevator.setState(clearanceMode);
//                     if (!elevator.isAtTarget()) break; // wait until we have clearance
//                 }

//                 // Now safe to stow arm & intake
//                 arm.setState(ArmMode.INTAKE);
//                 intake.setState(IntakeMode.Home);
//                 if (arm.isAtTarget() && intake.isAtTarget()) {
//                     // After STOW, go toward final targets
//                     phase = Phase.MOVE_ELEVATOR_TO_FINAL; // ensure elevator goes to the requested final, not just
//                     // the clearance
//                     timer.reset();
//                 }
//                 break;
//             }

//             case AE_PARALLEL: {
//                 // No extra safety bumps here; minimal fast path
//                 if (eDiff) elevator.setState(to.elevator);
//                 if (aDiff) arm.setState(to.arm);

//                 if ((!eDiff || elevator.isAtTarget()) && (!aDiff || arm.isAtTarget())) {
//                     phase = Phase.MOVE_INTAKE;
//                     timer.reset();
//                 }
//                 break;
//             }

//             case MOVE_ELEVATOR:
//                 if (eDiff) elevator.setState(to.elevator);
//                 if (!eDiff || elevator.isAtTarget()) {
//                     phase = Phase.MOVE_ARM;
//                     timer.reset();
//                 }
//                 break;

//             case MOVE_ARM:
//                 if (aDiff) arm.setState(to.arm);
//                 if (!aDiff || arm.isAtTarget()) {
//                     phase = Phase.MOVE_INTAKE;
//                     timer.reset();
//                 }
//                 break;

//             case MOVE_INTAKE:
//                 if (iDiff) intake.setState(to.intake);
//                 if (!iDiff || intake.isAtTarget()) {
//                     phase = Phase.MOVE_ELEVATOR_TO_FINAL; // always settle elevator at final requested height
//                     timer.reset();
//                 }
//                 break;

//             case MOVE_ELEVATOR_TO_FINAL:
//                 elevator.setState(to.elevator);
//                 if (elevator.isAtTarget()) {
//                     phase = Phase.HOLD;
//                     currentState = requestedState;
//                     currentGoal = to; // we're now holding the new goal
//                     timer.reset();
//                 }
//                 break;

//             case HOLD:
//             default:
//                 // Subsystems hold their setpoints internally
//                 break;
//         }
//     }

//     // ------------------ Helpers ------------------

//     /** Pick the smallest ElevatorMode whose targetDistance >= requiredInches. */
//     private ElevatorMode pickElevatorModeAtLeast(double requiredInches) {
//         ElevatorMode best = null;
//         double bestIn = Double.POSITIVE_INFINITY;

//         for (ElevatorMode m : ElevatorMode.values()) {
//             // ignore STOP/Home/INTAKE (0 in) as "clearance" modes
//             if (m == ElevatorMode.STOP || m == ElevatorMode.Home || m == ElevatorMode.INTAKE) continue;
//             Distance d = m.targetDistance;
//             if (d == null) continue; // defensive
//             double in = d.in(Inches);
//             if (in + 1e-6 >= requiredInches && in < bestIn) {
//                 bestIn = in;
//                 best = m;
//             }
//         }
//         return best;
//     }
// }
