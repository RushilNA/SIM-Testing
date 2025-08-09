package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorMode;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeMode;

import java.util.List;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

public class Superstructure extends SubsystemBase {
  /** Bundles each subsystem’s “mode” for a global state */
  public enum State {
    IDLE       (ArmMode.STOP,    ElevatorMode.STOP,    IntakeMode.STOP),
    INTAKE     (ArmMode.INTAKE,  ElevatorMode.INTAKE,  IntakeMode.INTAKE),
    L1_SCORING (ArmMode.L1,      ElevatorMode.L1,      IntakeMode.STOP),
    L2_SCORING (ArmMode.L2,      ElevatorMode.L2,      IntakeMode.STOP),
    L3_SCORING (ArmMode.L3,      ElevatorMode.L3,      IntakeMode.STOP),
    L4_SCORING (ArmMode.L4,      ElevatorMode.L4,      IntakeMode.STOP);

    public final ArmMode      armMode;
    public final ElevatorMode elevatorMode;
    public final IntakeMode   intakeMode;

    State(ArmMode a, ElevatorMode e, IntakeMode i) {
      this.armMode      = a;
      this.elevatorMode = e;
      this.intakeMode   = i;
    }
  }

  /** Operator “desires” that drive transitions */
  public static class SuperstructureInputs {
    public boolean wantIdle       = false;
    public boolean wantIntake     = false;
    public boolean wantL1Scoring  = false;
    public boolean wantL2Scoring  = false;
    public boolean wantL3Scoring  = false;
    public boolean wantL4Scoring  = false;
  }

  private static class Transition {
    final State            cur, next;
    final Runnable         enterFunction;
    final BooleanSupplier  transitionCheck;

    Transition(State cur, State next, BooleanSupplier check) {
      this(cur, next, ()->{}, check);
    }
    Transition(State cur, State next, Runnable enterFn, BooleanSupplier check) {
      this.cur            = cur;
      this.next           = next;
      this.enterFunction  = enterFn;
      this.transitionCheck = check;
    }
  }

  private final Arm      arm;
  private final Elevator elevator;
  private final Intake   intake;
  public final SuperstructureInputs inputs = new SuperstructureInputs();

  /** All of your FSM’s transitions, in priority order */
  private final List<Transition> transitions = Arrays.asList(
    // IDLE ↔ INTAKE
    new Transition(State.IDLE,   State.INTAKE,   () -> inputs.wantIntake),
    new Transition(State.INTAKE, State.IDLE,     () -> !inputs.wantIntake),

    // IDLE ↔ L1
    new Transition(State.IDLE,   State.L1_SCORING, () -> inputs.wantL1Scoring),
    new Transition(State.L1_SCORING, State.IDLE,   () -> !inputs.wantL1Scoring),

    // IDLE ↔ L2
    new Transition(State.IDLE,   State.L2_SCORING, () -> inputs.wantL2Scoring),
    new Transition(State.L2_SCORING, State.IDLE,   () -> !inputs.wantL2Scoring),

    // IDLE ↔ L3
    new Transition(State.IDLE,   State.L3_SCORING, () -> inputs.wantL3Scoring),
    new Transition(State.L3_SCORING, State.IDLE,   () -> !inputs.wantL3Scoring),

    // IDLE ↔ L4
    new Transition(State.IDLE,   State.L4_SCORING, () -> inputs.wantL4Scoring),
    new Transition(State.L4_SCORING, State.IDLE,   () -> !inputs.wantL4Scoring)
  );

  private final Timer stateTimer = new Timer();
  private State       state      = State.IDLE;

  public Superstructure(Arm arm, Elevator elevator, Intake intake) {
    this.arm      = arm;
    this.elevator = elevator;
    this.intake   = intake;
  }

  /**
   * (Optional) Manually force a state; resets timer & writes out to subsystems.
   */
  public void setState(State newState) {
    if (newState != state) {
      state = newState;
      stateTimer.reset();
      setStates();
    }
  }

  @Override
  public void periodic() {
    // Run the first matching transition
    for (Transition t : transitions) {
      if (t.cur == state && t.transitionCheck.getAsBoolean()) {
        state = t.next;
        t.enterFunction.run();
        stateTimer.reset();
        setStates();
        break;
      }
    }
  }

  /** Push the current State’s modes down into each subsystem */
  private void setStates() {
    arm.setState(state.armMode);
    elevator.setState(state.elevatorMode);
    intake.setState(state.intakeMode);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Superstructure");
    builder.addStringProperty(
      "State",
      () -> state.name(),
      null
    );
    builder.addBooleanProperty(
      "wantIntake",
      () -> inputs.wantIntake,
      v -> inputs.wantIntake = v
    );
    builder.addBooleanProperty(
      "wantL1",
      () -> inputs.wantL1Scoring,
      v -> inputs.wantL1Scoring = v
    );
    builder.addBooleanProperty(
      "wantL2",
      () -> inputs.wantL2Scoring,
      v -> inputs.wantL2Scoring = v
    );
    builder.addBooleanProperty(
      "wantL3",
      () -> inputs.wantL3Scoring,
      v -> inputs.wantL3Scoring = v
    );
    builder.addBooleanProperty(
      "wantL4",
      () -> inputs.wantL4Scoring,
      v -> inputs.wantL4Scoring = v
    );
  }
}
