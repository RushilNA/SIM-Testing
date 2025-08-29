// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.units.measure.*;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Intake.IntakeSubsytem; // intake pivot + rollers
// // expects: setAngle(Angle), getAngle(), setClawRoller(double)
// import frc.robot.subsystems.arm.armtest; // your concrete arm class
// import frc.robot.subsystems.elevator.ElevatorSubsystem; // expects: setHeight(Distance), getHeight()

// /**
//  * Superstructure (v3) — Arm + Elevator + Intake (with INTAKE PIVOT control & CURRENT-SENSE TRIGGER)
//  *
//  * <p>Update: Uses roller motor current (Phoenix6) to detect a captured piece instead of CANrange. - Debouncing is
//  * handled in IntakeSubsytem.hasGamePiece(). - On detection, we rumble the controller and auto-begin transfer.
//  */
// public class Superstructure extends SubsystemBase {
//     public enum Piece {
//         CORAL,
//         ALGAE
//     }

//     public enum State {
//         IDLE,

//         // Coral acquisition & transfer from Intake -> Arm
//         INTAKE_CORAL_DEPLOY, // Intake pivots out; arm/elevator at ground-intake pose
//         PREP_TRANSFER, // Intake pivots to transfer; elevator slightly above; arm down
//         ARM_GRAB_FROM_TRANSFER, // Arm rollers inward to take coral from intake
//         ARM_UP_WITH_CORAL, // Arm pivots up (claw faces up)
//         RETRACT_INTAKE_AND_STOW, // Intake stows; elevator to stow; arm up for CoM

//         // Scoring coral
//         PREP_SCORE_L2,
//         SCORE_L2,
//         PREP_SCORE_L3,
//         SCORE_L3,

//         // Algae handling
//         PREP_ALGAE_L2,
//         SCORE_ALGAE_L2,
//         PREP_ALGAE_L3,
//         SCORE_ALGAE_L3,

//         // Barge
//         PREP_BARGE,
//         RELEASE_BARGE,

//         // Parking / general
//         STOW
//     }

//     // --------------- Dependencies -----------------
//     private final armtest arm;
//     private final ElevatorSubsystem elevator;
//     private final IntakeSubsytem intake;
//     private final Timer rumbleTimer = new Timer();
//     private boolean rumbling = false;
//     private final CommandXboxController driver;

//     // --------------- State & helpers ---------------
//     private State state = State.IDLE;
//     private Piece carried = Piece.CORAL; // defaults; switched by API

//     private final Timer stateTimer = new Timer();

//     public Superstructure(armtest arm, ElevatorSubsystem elevator, IntakeSubsytem intake) {
//         this(arm, elevator, intake, null);
//     }

//     public Superstructure(
//             armtest arm, ElevatorSubsystem elevator, IntakeSubsytem intake, CommandXboxController driver) {
//         this.arm = arm;
//         this.elevator = elevator;
//         this.intake = intake;
//         this.driver = driver;
//         stateTimer.restart();
//         rumbleTimer.stop();
//         rumbleTimer.reset();
//     }

//     // --------------- Public API (high-level intents) ---------------
//     public void requestIntakeCoral() {
//         carried = Piece.CORAL;
//         transitionTo(State.INTAKE_CORAL_DEPLOY);
//     }

//     public void requestTransferToArm() {
//         transitionTo(State.PREP_TRANSFER);
//     }

//     public void requestStow() {
//         transitionTo(State.STOW);
//     }

//     public void requestScoreL2Coral() {
//         carried = Piece.CORAL;
//         transitionTo(State.PREP_SCORE_L2);
//     }

//     public void requestScoreL3Coral() {
//         carried = Piece.CORAL;
//         transitionTo(State.PREP_SCORE_L3);
//     }

//     public void requestScoreL2Algae() {
//         carried = Piece.ALGAE;
//         transitionTo(State.PREP_ALGAE_L2);
//     }

//     public void requestScoreL3Algae() {
//         carried = Piece.ALGAE;
//         transitionTo(State.PREP_ALGAE_L3);
//     }

//     public void requestBarge() {
//         transitionTo(State.PREP_BARGE);
//     }

//     // --------------- Periodic loop ---------------
//     @Override
//     public void periodic() {
//         switch (state) {
//             case IDLE -> {
//                 holdCurrent();
//                 updateRumble();
//             }

//             case INTAKE_CORAL_DEPLOY -> {
//                 // Intake pivots out to the floor pickup angle; arm down; elevator at ground height.
//                 setIntakePivot(Constants.INTAKE_DEPLOY_ANGLE);
//                 setIntakeSpeed(Constants.INTAKE_ROLLER_IN);
//                 arm.setAngle(Constants.ARM_GROUND_INTAKE_ANGLE);
//                 elevator.setHeight(Constants.ELEVATOR_GROUND_INTAKE_HEIGHT);

//                 // If object detected by current-sense, start transfer automatically and rumble controller.
//                 if (intake.hasGamePiece()) {
//                     startRumble(Constants.RUMBLE_TIME_S);
//                     transitionTo(State.PREP_TRANSFER);
//                 }
//                 updateRumble();
//             }

//             case PREP_TRANSFER -> {
//                 // Intake to transfer pose; elevator slightly above transfer; arm pointing down to receive.
//                 setIntakePivot(Constants.INTAKE_TRANSFER_ANGLE);
//                 setIntakeSpeed(Constants.INTAKE_ROLLER_IN_HOLD);
//                 elevator.setHeight(Constants.ELEVATOR_ABOVE_TRANSFER);
//                 arm.setAngle(Constants.ARM_TRANSFER_DOWN);
//                 if (stateTimer.hasElapsed(Constants.INTAKE_PIVOT_SETTLE_S)
//                         && atHeight(Constants.ELEVATOR_ABOVE_TRANSFER)
//                         && atAngle(Constants.ARM_TRANSFER_DOWN)) {
//                     transitionTo(State.ARM_GRAB_FROM_TRANSFER);
//                 }
//                 updateRumble();
//             }

//             case ARM_GRAB_FROM_TRANSFER -> {
//                 // Elevator drops to transfer height; arm rollers pull inward to capture from intake.
//                 setIntakePivot(Constants.INTAKE_TRANSFER_ANGLE);
//                 setIntakeSpeed(Constants.INTAKE_ROLLER_IN_HOLD);
//                 elevator.setHeight(Constants.ELEVATOR_TRANSFER_HEIGHT);
//                 arm.setClawRoller(+Constants.CLAW_SPEED_IN);
//                 if (atHeight(Constants.ELEVATOR_TRANSFER_HEIGHT) && stateTimer.hasElapsed(0.15)) {
//                     transitionTo(State.ARM_UP_WITH_CORAL);
//                 }
//                 updateRumble();
//             }

//             case ARM_UP_WITH_CORAL -> {
//                 // Rotate arm up so claw faces up. Keep holding inwards briefly during motion.
//                 arm.setAngle(Constants.ARM_TRANSFER_UP);
//                 arm.setClawRoller(+Constants.CLAW_SPEED_IN);
//                 if (atAngle(Constants.ARM_TRANSFER_UP)) {
//                     transitionTo(State.RETRACT_INTAKE_AND_STOW);
//                 }
//                 updateRumble();
//             }

//             case RETRACT_INTAKE_AND_STOW -> {
//                 // Stow intake; go to safe carry posture (arm up, elevator down).
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setClawRoller(0.0);
//                 arm.setAngle(Constants.ARM_CLAW_UP_CARRY);
//                 elevator.setHeight(Constants.ELEVATOR_STOW_HEIGHT);
//                 if (stateTimer.hasElapsed(Constants.INTAKE_PIVOT_SETTLE_S)
//                         && atAngle(Constants.ARM_CLAW_UP_CARRY)
//                         && atHeight(Constants.ELEVATOR_STOW_HEIGHT)) {
//                     transitionTo(State.STOW);
//                 }
//                 updateRumble();
//             }

//             case PREP_SCORE_L2 -> {
//                 // Keep intake stowed during scoring to avoid interference.
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setAngle(Constants.ARM_L2_ANGLE);
//                 elevator.setHeight(Constants.ELEVATOR_L2_HEIGHT);
//                 arm.setClawRoller(+Constants.CLAW_SPEED_IN); // hold until score
//                 if (atAngle(Constants.ARM_L2_ANGLE) && atHeight(Constants.ELEVATOR_L2_HEIGHT)) {
//                     transitionTo(State.SCORE_L2);
//                 }
//                 updateRumble();
//             }

//             case SCORE_L2 -> {
//                 arm.setAngle(Constants.ARM_L2_ANGLE.minus(Constants.SCORE_PITCH_DELTA));
//                 elevator.setHeight(Constants.ELEVATOR_L2_HEIGHT.plus(Constants.SCORE_NUDGE_DELTA));
//                 arm.setClawRoller(-Constants.CLAW_SPEED_OUT);
//                 if (stateTimer.hasElapsed(Constants.EJECT_TIME_S)) {
//                     arm.setClawRoller(0.0);
//                     transitionTo(State.STOW);
//                 }
//                 updateRumble();
//             }

//             case PREP_SCORE_L3 -> {
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setAngle(Constants.ARM_L3_ANGLE);
//                 elevator.setHeight(Constants.ELEVATOR_L3_HEIGHT);
//                 arm.setClawRoller(+Constants.CLAW_SPEED_IN);
//                 if (atAngle(Constants.ARM_L3_ANGLE) && atHeight(Constants.ELEVATOR_L3_HEIGHT)) {
//                     transitionTo(State.SCORE_L3);
//                 }
//                 updateRumble();
//             }

//             case SCORE_L3 -> {
//                 arm.setAngle(Constants.ARM_L3_ANGLE.minus(Constants.SCORE_PITCH_DELTA));
//                 elevator.setHeight(Constants.ELEVATOR_L3_HEIGHT.plus(Constants.SCORE_NUDGE_DELTA));
//                 arm.setClawRoller(-Constants.CLAW_SPEED_OUT);
//                 if (stateTimer.hasElapsed(Constants.EJECT_TIME_S)) {
//                     arm.setClawRoller(0.0);
//                     transitionTo(State.STOW);
//                 }
//                 updateRumble();
//             }

//             case PREP_ALGAE_L2 -> {
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setAngle(Constants.ARM_PARALLEL_TO_ELEVATOR);
//                 elevator.setHeight(Constants.ELEVATOR_ALGAE_L2_HEIGHT);
//                 arm.setClawRoller(+Constants.CLAW_SPEED_IN);
//                 if (atAngle(Constants.ARM_PARALLEL_TO_ELEVATOR) && atHeight(Constants.ELEVATOR_ALGAE_L2_HEIGHT)) {
//                     transitionTo(State.SCORE_ALGAE_L2);
//                 }
//                 updateRumble();
//             }

//             case SCORE_ALGAE_L2 -> {
//                 arm.setClawRoller(-Constants.CLAW_SPEED_OUT);
//                 if (stateTimer.hasElapsed(Constants.EJECT_TIME_S)) {
//                     arm.setClawRoller(0.0);
//                     // After storing algae, elevator down & arm up for CoM like coral
//                     setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                     setIntakeSpeed(0.0);
//                     elevator.setHeight(Constants.ELEVATOR_STOW_HEIGHT);
//                     arm.setAngle(Constants.ARM_CLAW_UP_CARRY);
//                     transitionTo(State.STOW);
//                 }
//                 updateRumble();
//             }

//             case PREP_ALGAE_L3 -> {
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setAngle(Constants.ARM_PARALLEL_TO_ELEVATOR);
//                 elevator.setHeight(Constants.ELEVATOR_ALGAE_L3_HEIGHT);
//                 arm.setClawRoller(+Constants.CLAW_SPEED_IN);
//                 if (atAngle(Constants.ARM_PARALLEL_TO_ELEVATOR) && atHeight(Constants.ELEVATOR_ALGAE_L3_HEIGHT)) {
//                     transitionTo(State.SCORE_ALGAE_L3);
//                 }
//                 updateRumble();
//             }

//             case SCORE_ALGAE_L3 -> {
//                 arm.setClawRoller(-Constants.CLAW_SPEED_OUT);
//                 if (stateTimer.hasElapsed(Constants.EJECT_TIME_S)) {
//                     arm.setClawRoller(0.0);
//                     setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                     setIntakeSpeed(0.0);
//                     elevator.setHeight(Constants.ELEVATOR_STOW_HEIGHT);
//                     arm.setAngle(Constants.ARM_CLAW_UP_CARRY);
//                     transitionTo(State.STOW);
//                 }
//                 updateRumble();
//             }

//             case PREP_BARGE -> {
//                 // Arm slight up-angle; elevator going up; keep intake stowed.
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setAngle(Constants.ARM_BARGE_ANGLE);
//                 elevator.setHeight(Constants.ELEVATOR_BARGE_HEIGHT);
//                 if (approachingHeight(Constants.ELEVATOR_BARGE_HEIGHT, Constants.BARGE_RELEASE_WINDOW)) {
//                     transitionTo(State.RELEASE_BARGE);
//                 }
//                 updateRumble();
//             }

//             case RELEASE_BARGE -> {
//                 arm.setClawRoller(-Constants.CLAW_SPEED_OUT);
//                 if (stateTimer.hasElapsed(Constants.EJECT_TIME_S)) {
//                     arm.setClawRoller(0.0);
//                     transitionTo(State.STOW);
//                 }
//                 updateRumble();
//             }

//             case STOW -> {
//                 setIntakePivot(Constants.INTAKE_STOW_ANGLE);
//                 setIntakeSpeed(0.0);
//                 arm.setAngle(Constants.ARM_CLAW_UP_CARRY);
//                 elevator.setHeight(Constants.ELEVATOR_STOW_HEIGHT);
//                 arm.setClawRoller(0.0);
//                 updateRumble();
//             }
//         }
//     }

//     // --------------- Internals ---------------
//     private void holdCurrent() {
//         /* keep last commands */
//     }

//     private void transitionTo(State next) {
//         if (state != next) {
//             state = next;
//             stateTimer.restart();
//         }
//     }

//     private boolean atAngle(Angle target) {
//         Angle current = arm.getAngle();
//         return Math.abs(current.minus(target).in(Degrees)) <= Constants.ANGLE_EPS.in(Degrees);
//     }

//     private boolean atHeight(Distance target) {
//         Distance current = elevator.getHeight();
//         return Math.abs(current.minus(target).in(Meters)) <= Constants.HEIGHT_EPS.in(Meters);
//     }

//     private boolean approachingHeight(Distance target, Distance window) {
//         Distance current = elevator.getHeight();
//         return current.in(Meters) >= (target.minus(window).in(Meters));
//     }

//     private void setIntakePivot(Angle angle) {
//         // Convert desired PIVOT ANGLE (deg) -> TalonFX ROTATIONS.
//         // rot = (deg / 360) * gearRatio + zeroOffset
//         double rotations = (angle.in(Degrees) / 360.0) * Constants.INTAKE_PIVOT_GEAR_RATIO
//                 + Constants.INTAKE_PIVOT_ZERO_OFFSET_ROT;
//         intake.setIntakePivoit(rotations);
//     }

//     private void startRumble(double durationSec) {
//         if (driver == null) return;
//         driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
//         rumbling = true;
//         rumbleTimer.restart();
//     }

//     private void updateRumble() {
//         if (driver == null) return;
//         if (rumbling && rumbleTimer.hasElapsed(Constants.RUMBLE_TIME_S)) {
//             driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
//             rumbling = false;
//         }
//     }

//     private void setIntakeSpeed(double speed) {
//         intake.setIntakeSpeed(speed);
//     }

//     // --------------- Constants to FILL IN ---------------
//     public static final class Constants {
//         // ---- Intake pivot angles (degrees) ----
//         public static final Angle INTAKE_STOW_ANGLE = Degrees.of(-10); // tucked in
//         public static final Angle INTAKE_DEPLOY_ANGLE = Degrees.of(-85); // out to floor
//         public static final Angle INTAKE_TRANSFER_ANGLE = Degrees.of(-40); // handoff pose
//         public static final double INTAKE_PIVOT_SETTLE_S = 0.20; // wait time if no sensor

//         public static final double INTAKE_PIVOT_GEAR_RATIO = 1.0; // motor revs per arm rev
//         public static final double INTAKE_PIVOT_ZERO_OFFSET_ROT = 0.0; // mechanical zero offset in motor rotations

//         // Intake roller speeds
//         public static final double INTAKE_ROLLER_IN = 0.8;
//         public static final double INTAKE_ROLLER_IN_HOLD = 0.2;

//         // ---- Core setpoints (replace with your actual numbers) ----
//         public static final Angle ARM_GROUND_INTAKE_ANGLE = Degrees.of(-80); // arm points down
//         public static final Distance ELEVATOR_GROUND_INTAKE_HEIGHT = Meters.of(0.05);

//         public static final Angle ARM_TRANSFER_DOWN = Degrees.of(-60); // arm points down at transfer
//         public static final Angle ARM_TRANSFER_UP = Degrees.of(+60); // claw faces up after rotate
//         public static final Distance ELEVATOR_ABOVE_TRANSFER = Meters.of(0.65); // slightly above transfer
//         public static final Distance ELEVATOR_TRANSFER_HEIGHT = Meters.of(0.55); // where elevator receives

//         public static final Angle ARM_CLAW_UP_CARRY = Degrees.of(+70); // safe carry angle (claw up)
//         public static final Distance ELEVATOR_STOW_HEIGHT = Meters.of(0.15); // low for CoM

//         // L2 / L3 coral
//         public static final Angle ARM_L2_ANGLE = Degrees.of(+20);
//         public static final Distance ELEVATOR_L2_HEIGHT = Meters.of(0.75);
//         public static final Angle ARM_L3_ANGLE = Degrees.of(+35);
//         public static final Distance ELEVATOR_L3_HEIGHT = Meters.of(1.25);

//         // Algae with arm parallel to elevator
//         public static final Angle ARM_PARALLEL_TO_ELEVATOR = Degrees.of(0);
//         public static final Distance ELEVATOR_ALGAE_L2_HEIGHT = Meters.of(0.70);
//         public static final Distance ELEVATOR_ALGAE_L3_HEIGHT = Meters.of(1.10);

//         // Barge
//         public static final Angle ARM_BARGE_ANGLE = Degrees.of(+15); // slight up angle
//         public static final Distance ELEVATOR_BARGE_HEIGHT = Meters.of(1.00);
//         public static final Distance BARGE_RELEASE_WINDOW = Meters.of(0.05); // release when within 5 cm

//         // Scoring nudge deltas
//         public static final Angle SCORE_PITCH_DELTA = Degrees.of(6); // arm tip-down when scoring
//         public static final Distance SCORE_NUDGE_DELTA = Meters.of(0.03); // elevator small push

//         // Claw rollers (unitless +in / -out)
//         public static final double CLAW_SPEED_IN = 0.25; // inward to hold
//         public static final double CLAW_SPEED_OUT = 0.5; // outward to release

//         // Timing
//         public static final double EJECT_TIME_S = 0.25;
//         public static final double RUMBLE_TIME_S = 0.25;

//         // Tolerances
//         public static final Angle ANGLE_EPS = Degrees.of(2.0);
//         public static final Distance HEIGHT_EPS = Meters.of(0.01);
//     }
// }
