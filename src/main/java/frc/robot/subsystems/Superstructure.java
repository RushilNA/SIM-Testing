package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.git;
import frc.robot.git.gitoutput;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.wantedState;
import frc.robot.subsystems.UpperBoddy.Pose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** 2910-style Superstructure coordinating Intake + Pivoit + Uperbody + Arm rollers. */
public class Superstructure extends edu.wpi.first.wpilibj2.command.SubsystemBase {

    // ───────── Subsystems ─────────
    private final UpperBoddy uperbody;
    private final Flywheel intake; // your coral floor intake (with beam-break)
    private final Intake pivoit; // your pivot thing that presents coral to arm
    private final Armwheels arm; // arm rollers (score + algae intake)
    private final Drive drive;

    public enum WantedSuperState {
        STOPPED,
        DEFAULT_STATE, // stow pipeline

        // Intake & handoff
        INTAKE_CORAL,
        HANDOFF_CORAL_TO_ARM,

        // Manual scoring (driver presses “Score Now” to outtake via ARM rollers)
        SCORE_L1_LEFT_HIGH,
        SCORE_L1_LEFT_LOW,
        SCORE_L1_RIGHT_HIGH,
        SCORE_L1_RIGHT_LOW,
        SCORE_L2_LEFT,
        SCORE_L2_RIGHT,
        SCORE_L3_LEFT,
        SCORE_L3_RIGHT,

        // Manual absolute (direct pose)
        MANUAL_DIRECT,

        // Chooser-driven actions
        MOVE_TO_SELECTED, // go to pose from chooser (no outtake)
        SCORE_NOW, // force immediate scoring using selected pose

        // NEW
        BARGE_SHOOT,
        INTAKE_ALGAE_L2,
        INTAKE_ALGAE_L3
    }

    public enum CurrentSuperState {
        STOPPED,
        STOWING,
        INTAKING,
        HANDOFF,
        SCORING,
        MANUAL_DIRECT,
        TARGETING, // chooser pose, no outtake
        // NEW
        BARGE,
        ALGAE_INTAKE
    }

    private WantedSuperState wanted = WantedSuperState.STOPPED;
    private CurrentSuperState current = CurrentSuperState.STOPPED;
    private CurrentSuperState previous = CurrentSuperState.STOPPED;

    private final SendableChooser<WantedSuperState> scoreChooser = new SendableChooser<>();
    private WantedSuperState lastChosenScore = null;

    private boolean coral = false;

    // Allow controller or buttons to latch “score now”
    private boolean scoreNowLatched = false;

    public void setScoreNow(boolean v) {
        scoreNowLatched = v;
        Logger.recordOutput("SS/ScoreNowLatched", scoreNowLatched);
    }

    // ───────── AdvantageKit Logged Mechanism2d ─────────
    private final LoggedMechanism2d lmech = new LoggedMechanism2d(3.0, 2.0); // width x height (m)
    private final LoggedMechanismRoot2d lbase = lmech.getRoot("Base", 1.5, 0.5);

    // Elevator from base
    private final LoggedMechanismLigament2d lelev = new LoggedMechanismLigament2d("Elevator", 0.5, 90.0);
    // Shoulder from elevator top
    private final LoggedMechanismLigament2d lshoulder = new LoggedMechanismLigament2d("Shoulder", 0.40, 180);
    // Roller from shoulder tip (cosmetic)
    private final LoggedMechanismLigament2d lroller = new LoggedMechanismLigament2d("Roller", 0.12, 0.0);
    // Pivot indicator at base
    private final LoggedMechanismLigament2d lpivot = new LoggedMechanismLigament2d("Pivot", 0.25, 90.0);

    public Superstructure(UpperBoddy uperbody, Flywheel intake, Intake pivoit, Armwheels arm, Drive drive) {
        this.uperbody = uperbody;
        this.intake = intake;
        this.pivoit = pivoit;
        this.arm = arm;
        this.drive = drive;

        // hook up the tree
        lbase.append(lelev);
        lelev.append(lshoulder);
        lshoulder.append(lroller);
        lbase.append(lpivot);

        Logger.recordOutput("Mechanism2d/Superstructure", lmech);

        // Chooser options (unchanged)
        scoreChooser.setDefaultOption("L2 Left", WantedSuperState.SCORE_L2_LEFT);
        scoreChooser.addOption("L2 Right", WantedSuperState.SCORE_L2_RIGHT);
        scoreChooser.addOption("L3 Left", WantedSuperState.SCORE_L3_LEFT);
        scoreChooser.addOption("L3 Right", WantedSuperState.SCORE_L3_RIGHT);
        scoreChooser.addOption("L1 Left (High)", WantedSuperState.SCORE_L1_LEFT_HIGH);
        scoreChooser.addOption("L1 Left (Low)", WantedSuperState.SCORE_L1_LEFT_LOW);
        scoreChooser.addOption("L1 Right (High)", WantedSuperState.SCORE_L1_RIGHT_HIGH);
        scoreChooser.addOption("L1 Right (Low)", WantedSuperState.SCORE_L1_RIGHT_LOW);
        scoreChooser.addOption("Algae Intake L2", WantedSuperState.INTAKE_ALGAE_L2);
        scoreChooser.addOption("Algae Intake L3", WantedSuperState.INTAKE_ALGAE_L3);
        scoreChooser.addOption("Barge", WantedSuperState.BARGE_SHOOT);

        Shuffleboard.getTab("Superstructure")
                .add("Score Target", scoreChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withPosition(0, 1)
                .withSize(3, 1);

        Shuffleboard.getTab("Superstructure")
                .add("Move To Selected", setStateCommand(WantedSuperState.MOVE_TO_SELECTED))
                .withPosition(3, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("Superstructure")
                .add("Score Selected", setStateCommand(WantedSuperState.SCORE_NOW))
                .withPosition(5, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);

        // Quick buttons for the new flows (optional)
        Shuffleboard.getTab("Superstructure")
                .add("Barge Shoot", setStateCommand(WantedSuperState.BARGE_SHOOT))
                .withPosition(7, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("Superstructure")
                .add("Algae Intake L2", setStateCommand(WantedSuperState.INTAKE_ALGAE_L2))
                .withPosition(9, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);

        Shuffleboard.getTab("Superstructure")
                .add("Algae Intake L3", setStateCommand(WantedSuperState.INTAKE_ALGAE_L3))
                .withPosition(11, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kCommand);
    }

    // ───────── Shuffleboard UI ─────────
    private final GenericEntry scoreNowEntry = Shuffleboard.getTab("Superstructure")
            .add("Score Now (Hold/RT)", false)
            .withPosition(0, 0)
            .getEntry();
    private final GenericEntry pivotUpManualEntry = Shuffleboard.getTab("Superstructure")
            .add("Pivot Up (Manual)", false)
            .withPosition(1, 0)
            .getEntry();
    private final GenericEntry manualModeEntry = Shuffleboard.getTab("Superstructure")
            .add("Manual Mode (Offsets)", false)
            .withPosition(2, 0)
            .getEntry();
    private final GenericEntry manualElevDeltaEntry = Shuffleboard.getTab("Superstructure")
            .add("Manual Elev Δ (m)", 0.0)
            .withPosition(3, 0)
            .getEntry();
    private final GenericEntry manualShoulderDeltaDegEntry = Shuffleboard.getTab("Superstructure")
            .add("Manual Shoulder Δ (deg)", 0.0)
            .withPosition(4, 0)
            .getEntry();
    private final GenericEntry manualElevAbsEntry = Shuffleboard.getTab("Superstructure")
            .add("Manual Elev ABS (m)", 0.20)
            .withPosition(5, 0)
            .getEntry();
    private final GenericEntry manualShoulderAbsDegEntry = Shuffleboard.getTab("Superstructure")
            .add("Manual Shoulder ABS (deg)", 45.0)
            .withPosition(6, 0)
            .getEntry();
    private final GenericEntry HAndoff = Shuffleboard.getTab("Superstructure")
            .add("Auto Handoff", true)
            .withPosition(7, 0)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

    private final GenericEntry Algae = Shuffleboard.getTab("Superstructure")
            .add("Alage Detector", false)
            .withPosition(8, 1)
            .withSize(5, 5)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    // ───────── Poses / Tunables ─────────
    public static final class Poses {
        public static UpperBoddy.Pose pose(double elevMeters, Rotation2d shoulder) {
            return new UpperBoddy.Pose(Meters.of(elevMeters), Degrees.of(shoulder.getDegrees()));
        }

        public static double elevM(UpperBoddy.Pose p) {
            return p.elev().in(Meters);
        }

        public static Rotation2d shoulderR(UpperBoddy.Pose p) {
            return Rotation2d.fromDegrees(p.shoulder().in(Degrees));
        }

        private static final Rotation2d LEFT_OFF = Rotation2d.fromDegrees(0);
        private static final Rotation2d RIGHT_OFF = Rotation2d.fromDegrees(0);

        // STOW
        public static final UpperBoddy.Pose STOW_ARM_45 = pose(1.2, Rotation2d.fromDegrees(0));
        public static final UpperBoddy.Pose STOW_FINAL = pose(0.8, Rotation2d.fromDegrees(0));

        // Intake & Handoff
        public static final UpperBoddy.Pose INTAKE = pose(1.2, Rotation2d.fromDegrees(180));
        public static final UpperBoddy.Pose HANDOFF_ELEV_UP = pose(1.2, Rotation2d.fromDegrees(180));
        public static final UpperBoddy.Pose HANDOFF_ARM_ZERO = pose(1.2, Rotation2d.fromDegrees(180));
        public static final UpperBoddy.Pose HANDOFF_DROP = pose(0.8, Rotation2d.fromDegrees(180));

        // L1/L2/L3 bases
        public static final UpperBoddy.Pose L1_HIGH_BASE = pose(0.34, Rotation2d.fromDegrees(84));
        public static final UpperBoddy.Pose L1_LOW_BASE = pose(0.26, Rotation2d.fromDegrees(78));
        public static final UpperBoddy.Pose L2_BASE = pose(0.70, Rotation2d.fromDegrees(60));
        public static final UpperBoddy.Pose L3_BASE = pose(1.05, Rotation2d.fromDegrees(60));

        // Barge shoot pose (tune these!)
        public static final UpperBoddy.Pose BARGE = pose(1.65, Rotation2d.fromDegrees(95));

        // NEW: Algae intake poses (tune as needed)
        public static final UpperBoddy.Pose ALGAE_L2_INTAKE = pose(0.72, Rotation2d.fromDegrees(78));
        public static final UpperBoddy.Pose ALGAE_L3_INTAKE = pose(1.10, Rotation2d.fromDegrees(96));

        public static UpperBoddy.Pose withLeft(UpperBoddy.Pose p) {
            return pose(elevM(p), shoulderR(p).plus(LEFT_OFF));
        }

        public static UpperBoddy.Pose withRight(UpperBoddy.Pose p) {
            return pose(elevM(p), shoulderR(p).plus(RIGHT_OFF));
        }
    }

    private static final double kPoseSettleS = 0.15;
    private static final double kDuckElevDelta = -0.06;
    private static final Rotation2d L4kDuckArmDelta = Rotation2d.fromDegrees(0);
    private static final Rotation2d L3kDuckArmDelta = Rotation2d.fromDegrees(30);
    private static final Rotation2d L2kDuckArmDelta = Rotation2d.fromDegrees(45);
    private static final Rotation2d L1kDuckArmDelta = Rotation2d.fromDegrees(0);

    private static final double kDuckHoldS = 0.25;
    private static final double kStowAfterArm45HoldS = 0.15;
    private static final double kStowAfterElevDownS = 0.15;
    private static final double kPivotIntakeDeg = 90.0;
    private static final double kPivotHandoffDeg = 35.0;
    private static final double kPivotHoldDeg = 90.0;

    // ───────── Phase machines ─────────
    private enum StowPhase {
        ARM_TO_45,
        ELEVATOR_DOWN,
        ARM_TO_FINAL,
        DONE
    }

    private enum HandoffPhase {
        ELEV_UP,
        ARM_ZERO,
        WAIT_PIVOT_UP,
        ELEV_DROP,
        CAPTURE,
        DONE
    }

    private enum ScorePhase {
        MOVE,
        DUCK_OUTTAKE,
        RETREAT,
        TO_HANDOFF
    }

    // NEW: barge + algae phases
    private enum BargePhase {
        MOVE,
        SHOOT,
        TO_STOW
    }

    private enum AlgaePhase {
        MOVE,
        INTAKE_HOLD,
        TO_STOW
    }

    private enum CoralState {
        NONE,
        IN_INTAKE,
        IN_ARM
    }

    private CoralState coralState = CoralState.NONE;

    private StowPhase stowPhase = StowPhase.DONE;
    private HandoffPhase handoffPhase = HandoffPhase.DONE;
    private ScorePhase scorePhase = ScorePhase.MOVE;
    private BargePhase bargePhase = BargePhase.MOVE;
    private AlgaePhase algaePhase = AlgaePhase.MOVE;

    private double phaseStart = 0.0;
    private Pose lastScoringTarget = null;

    // ───────── Public API ─────────
    public void setWantedState(WantedSuperState s) {
        wanted = s;
    }

    public Command setStateCommand(WantedSuperState s) {
        return Commands.runOnce(() -> setWantedState(s));
    }

    @Override
    public void periodic() {
        previous = current;
        current = handleTransitions();
        applyStates();

        Logger.recordOutput("SS/Wanted", wanted.toString());
        Logger.recordOutput("SS/Current", current.toString());
        Logger.recordOutput("SS/ScorePhase", scorePhase.toString());
        Logger.recordOutput("SS/StowPhase", stowPhase.toString());
        Logger.recordOutput("SS/HandoffPhase", handoffPhase.toString());
        Logger.recordOutput("SS/BargePhase", bargePhase.toString());
        Logger.recordOutput("SS/AlgaePhase", algaePhase.toString());
        Logger.recordOutput("SS/ChosenScore", getChosenScore().toString());
        SmartDashboard.putBoolean("SS/Check", uperbody.reachedSetpoint());
        Logger.recordOutput("SS/HasCoral", uperbody.getpos());
        Logger.recordOutput("SS/Scorechooser", getChosenScore().toString());
        Logger.recordOutput("SS/Whereisgettingscored", getCurrentLevelSimple());
        double pivotDeg = uperbody.getpos(); // or however you read it
        double armYawRad = MathUtil.angleModulus(Units.degreesToRadians(pivotDeg));

        // If your pivot sensor already gives an Angle:

        // Wrist example (same idea)
        double wristDeg = uperbody.getpos(); // your value in degrees?
        double wristRad = MathUtil.angleModulus(Units.degreesToRadians(wristDeg));

        updateLoggedMechanism();
        Logger.recordOutput("Final Component intake", new Pose3d[] {
            new Pose3d(
                    0,
                    pivoit.getPosition().abs(Degrees) * -0.001,
                    pivoit.getPosition().abs(Degrees) * 0.0042,
                    new Rotation3d(pivoit.getPosition().baseUnitMagnitude(), 0, 0))
        });

        Logger.recordOutput(
                "Final Component elevator",
                new Pose3d[] {new Pose3d(0, 0, uperbody.getelevatorpos(), new Rotation3d(0, 0, 0))});

        Logger.recordOutput("Final Component arm", new Pose3d[] {
            new Pose3d(0, 0, uperbody.getelevatorpos(), new Rotation3d(0, Units.degreesToRadians(uperbody.getpos()), 0))
        });

        Logger.recordOutput(
                "Final Component thingy",
                new Pose3d[] {new Pose3d(0, 0, uperbody.getelevatorpos(), new Rotation3d(0, 0, 0))});

        ScoringKinematics.logViz(
                "VIZ",
                MathUtil.clamp(uperbody.getelevatorpos(), 0, 0.801),
                uperbody.getelevatorpos(),
                Units.degreesToRadians(uperbody.getpos()),
                pivoit.getPosition().in(Degrees));
        double pivotDegNow = pivoit.getPosition().in(Degrees);

        git.CoralLocation coralLoc =
                switch (coralState) {
                    case IN_INTAKE -> git.CoralLocation.IN_INTAKE;
                    case IN_ARM -> git.CoralLocation.IN_ARM;
                    case NONE -> git.CoralLocation.NONE;
                };

        Logger.recordOutput("SS/Coral Loc", coralLoc);

        final gitoutput kinimatic = git.computate(
                0,
                MathUtil.clamp(uperbody.getelevatorpos(), 0, MathUtil.clamp(uperbody.getelevatorpos() / 2.5, 0.3, 0.6)),
                uperbody.getelevatorpos(),
                uperbody.getpos(),
                getCurrentLevelSimple(),
                coralLoc);
        Logger.recordOutput("mechanismPoses", new Pose3d[] {
            new Pose3d(
                    0,
                    pivoit.getPosition().abs(Degrees) * -0.001,
                    pivoit.getPosition().abs(Degrees) * 0.0042,
                    new Rotation3d(pivoit.getPosition().baseUnitMagnitude(), 0, 0)),
            kinimatic.getElevatorPose(),
            kinimatic.getArmPose(),
            kinimatic.getCarriagePose()
        });

        Logger.recordOutput("SS/Coral", new Pose3d(drive.getPose()).transformBy(kinimatic.coralT));

        Logger.recordOutput("SS/Coral Rotation", kinimatic.coralT.getRotation());
    }

    public double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    private void updateLoggedMechanism() {
        UpperBoddy.Pose pose = uperbody.getMeasuredPose();
        if (pose == null) pose = uperbody.getTargetPose();

        double elevM = (pose != null) ? pose.elev().in(Meters) : 0.0;
        double shoulderDegAbs = (pose != null) ? pose.shoulder().in(Degrees) : 0.0;

        elevM = Math.max(0.0, Math.min(elevM, 1.5));

        lelev.setAngle(90.0);
        lelev.setLength(Math.max(0.2, elevM));
        lshoulder.setAngle(-shoulderDegAbs);

        boolean out = false, in = false;
        try {
            out = arm.isOuttakingFast();
            in = arm.isIntaking();
        } catch (Exception ignored) {
        }
        lroller.setLength(out ? 0.14 : (in ? 0.10 : 0.12));

        double pivotDeg = 0;
        double pivotDeg1 = 0;
        try {
            pivotDeg = pivoit.getPosition().abs(Degrees);
            pivotDeg1 = map(pivotDeg, 0, 90, 90, 0);
        } catch (Exception ignored) {
        }
        lpivot.setAngle(pivotDeg1);

        boolean usingPivot = (current == CurrentSuperState.INTAKING || current == CurrentSuperState.HANDOFF);
        lpivot.setLineWeight(usingPivot ? 4 : 2);

        Logger.recordOutput("Mech/ElevMeters", elevM);
        Logger.recordOutput("Mech/ShoulderAbsDeg", shoulderDegAbs);
        Logger.recordOutput("Mech/PivotDeg", pivotDeg);
        Logger.recordOutput("Mechanism2d/Superstructure", lmech);
    }

    // ───────── Transitions (2910 style) ─────────
    private CurrentSuperState handleTransitions() {
        return switch (wanted) {
            case STOPPED -> CurrentSuperState.STOPPED;
            case DEFAULT_STATE -> CurrentSuperState.STOWING;

            case INTAKE_CORAL -> {
                if (!intake.hasCoral()) yield CurrentSuperState.INTAKING;
                yield CurrentSuperState.HANDOFF;
            }
            case HANDOFF_CORAL_TO_ARM -> CurrentSuperState.HANDOFF;

            case SCORE_L1_LEFT_HIGH,
                    SCORE_L1_LEFT_LOW,
                    SCORE_L1_RIGHT_HIGH,
                    SCORE_L1_RIGHT_LOW,
                    SCORE_L2_LEFT,
                    SCORE_L2_RIGHT,
                    SCORE_L3_LEFT,
                    SCORE_L3_RIGHT -> CurrentSuperState.SCORING;

            case MANUAL_DIRECT -> CurrentSuperState.MANUAL_DIRECT;
            case MOVE_TO_SELECTED -> {
                WantedSuperState chosen = getChosenScore();
                if (chosen == WantedSuperState.INTAKE_ALGAE_L2 || chosen == WantedSuperState.INTAKE_ALGAE_L3) {
                    yield CurrentSuperState.ALGAE_INTAKE;
                } else {
                    yield CurrentSuperState.TARGETING;
                }
            }

                // NEW: route SCORE_NOW based on chooser
            case SCORE_NOW -> {
                WantedSuperState chosen = getChosenScore();
                if (chosen == WantedSuperState.BARGE_SHOOT) {
                    yield CurrentSuperState.BARGE; // go straight to barge pipeline
                } else {
                    yield CurrentSuperState.SCORING; // normal coral scoring pipeline
                }
            }

            case BARGE_SHOOT -> CurrentSuperState.BARGE;
            case INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 -> CurrentSuperState.ALGAE_INTAKE;
        };
    }

    // ───────── Apply outputs ─────────
    private void applyStates() {
        if (previous != current) onEnter(current);

        switch (current) {
            case STOPPED -> {
                intake.stop();
                arm.rollerStop();
            }
            case STOWING -> runStowPipeline();

            case INTAKING -> {
                uperbody.setTargetPose(maybeOffsets(Poses.INTAKE));
                intake.collectCoral();
                if (!intake.hasCoral()) {
                    pivoit.setWantedState(wantedState.COLLECT_CORAL);
                } else {
                    pivoit.Handoff();
                    coralState = CoralState.IN_INTAKE;
                }
                arm.rollerStop();
            }

            case HANDOFF -> runHandoffPipeline();

            case TARGETING -> {
                UpperBoddy.Pose target = poseFromChooser();
                uperbody.setTargetPose(target);
                intake.stop();
                arm.rollerStop();
                pivoit.Home();
            }

            case SCORING -> runScoringPipeline();

                // NEW
            case BARGE -> runBargeShootPipeline();
            case ALGAE_INTAKE -> runAlgaeIntakePipeline();

            case MANUAL_DIRECT -> {
                UpperBoddy.Pose abs = Poses.pose(
                        manualElevAbsEntry.getDouble(0.20),
                        Rotation2d.fromDegrees(manualShoulderAbsDegEntry.getDouble(45.0)));
                uperbody.setTargetPose(abs);
                arm.rollerStop();
                intake.stop();
                pivoit.setTargetAngleDeg(kPivotHoldDeg);
            }
        }
    }

    private void onEnter(CurrentSuperState s) {
        phaseStart = Timer.getFPGATimestamp();

        switch (s) {
            case STOWING -> {
                stowPhase = StowPhase.ARM_TO_45;
                uperbody.setTargetPose(maybeOffsets(Poses.STOW_ARM_45));
                intake.stop();
                arm.rollerStop();
                pivoit.Home();
            }
            case HANDOFF -> {
                handoffPhase = HandoffPhase.ELEV_UP;
                uperbody.setTargetPose(maybeOffsets(Poses.HANDOFF_ELEV_UP));
                arm.rollerStop();
                pivoit.Handoff();
            }
            case SCORING -> {
                if (wanted == WantedSuperState.SCORE_NOW) scoreNowLatched = true;

                Pose base = poseFromChooser();
                lastScoringTarget = maybeOffsets(base);
                uperbody.setTargetPose(lastScoringTarget);

                intake.stop();
                arm.rollerStop();
                pivoit.Home();
                scorePhase = ScorePhase.MOVE;
            }

                // NEW: Barge shoot
            case BARGE -> {
                bargePhase = BargePhase.MOVE;
                uperbody.setTargetPose(Poses.BARGE);
                intake.stop();
                arm.rollerStop();
                pivoit.Home();
            }

                // NEW: Algae intake (L2/L3)
            case ALGAE_INTAKE -> {
                algaePhase = AlgaePhase.MOVE;
                UpperBoddy.Pose target =
                        (wanted == WantedSuperState.INTAKE_ALGAE_L2) ? Poses.ALGAE_L2_INTAKE : Poses.ALGAE_L3_INTAKE;
                uperbody.setTargetPose(target);
                intake.stop(); // floor intake not used
                arm.rollerStop();
                pivoit.Home();
            }

            default -> {}
        }
    }

    public double elevatorScalor() {
        double scalr = uperbody.getelevatorpos();

        return scalr;
    }

    // ───────── Pipelines ─────────
    // STOW
    private void runStowPipeline() {
        double t = Timer.getFPGATimestamp() - phaseStart;
        switch (stowPhase) {
            case ARM_TO_45 -> {
                pivoit.Handoff();
                if (uperbody.reachedSetpoint() && t >= kStowAfterArm45HoldS) {
                    uperbody.setTargetPose(
                            Poses.pose(Poses.elevM(Poses.STOW_FINAL), Poses.shoulderR(Poses.STOW_ARM_45)));
                    stowPhase = StowPhase.ELEVATOR_DOWN;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case ELEVATOR_DOWN -> {
                pivoit.Handoff();

                if (uperbody.reachedSetpoint() && t >= kStowAfterElevDownS) {
                    uperbody.setTargetPose(Poses.STOW_FINAL);
                    stowPhase = StowPhase.ARM_TO_FINAL;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case ARM_TO_FINAL -> {
                pivoit.Handoff();

                if (uperbody.reachedSetpoint()) stowPhase = StowPhase.DONE;
            }
            case DONE -> {
                pivoit.Home();

                /* idle */
            }
        }
    }

    // HANDOFF (intake feeds, arm pulls coral IN)
    private void runHandoffPipeline() {
        double t = Timer.getFPGATimestamp() - phaseStart;
        switch (handoffPhase) {
            case ELEV_UP -> {
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS) {
                    uperbody.setTargetPose(maybeOffsets(Poses.HANDOFF_ARM_ZERO));
                    handoffPhase = HandoffPhase.ARM_ZERO;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case ARM_ZERO -> {
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS) {
                    pivoit.Handoff();
                    handoffPhase = HandoffPhase.WAIT_PIVOT_UP;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case WAIT_PIVOT_UP -> {
                if (isPivotUp()) {
                    uperbody.setTargetPose(maybeOffsets(Poses.HANDOFF_DROP));
                    arm.rollerIn();

                    intake.setWantedState(frc.robot.subsystems.flywheel.Flywheel.wantedState.HOLDING);
                    pivoit.Handoff();

                    handoffPhase = HandoffPhase.ELEV_DROP;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case ELEV_DROP -> {
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS) {
                    handoffPhase = HandoffPhase.CAPTURE;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case CAPTURE -> {
                if (t >= 0.15) {
                    intake.stop();
                    arm.rollerStop();
                    handoffPhase = HandoffPhase.DONE;
                    uperbody.setTargetPose(Poses.HANDOFF_ELEV_UP);
                    coralState = CoralState.IN_ARM;
                }
            }
            case DONE -> {
                /* finished */
                if (uperbody.reachedSetpoint()) {
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                }
            }
        }
    }

    // SCORING (ARM rollers outtake; intake stays off)
    private void runScoringPipeline() {
        double t = Timer.getFPGATimestamp() - phaseStart;

        switch (scorePhase) {
            case MOVE -> {
                boolean scorePressed = scoreNowLatched || scoreNowEntry.getBoolean(false);
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS && scorePressed && lastScoringTarget != null) {
                    Rotation2d kDuckArmDelta = Rotation2d.fromDegrees(0);
                    kDuckArmDelta = (getCurrentLevelSimple() == "L4")
                            ? L4kDuckArmDelta
                            : (getCurrentLevelSimple() == "L3")
                                    ? L3kDuckArmDelta
                                    : (getCurrentLevelSimple() == "L2") ? L2kDuckArmDelta : L1kDuckArmDelta;

                    UpperBoddy.Pose duck = Poses.pose(
                            Poses.elevM(lastScoringTarget) + kDuckElevDelta,
                            Poses.shoulderR(lastScoringTarget).plus(kDuckArmDelta));
                    uperbody.setTargetPose(duck);

                    arm.rollerOutFast();
                    MapleSimUtils.scoreCoral(drive, uperbody, arm, getCurrentLevelSimple());
                    coralState = CoralState.NONE;

                    scorePhase = ScorePhase.DUCK_OUTTAKE;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case DUCK_OUTTAKE -> {
                if (uperbody.reachedSetpoint() && t >= kDuckHoldS) {

                    arm.rollerStop();
                    UpperBoddy.Pose retreat = Poses.pose(
                            Poses.elevM(lastScoringTarget),
                            Poses.shoulderR(lastScoringTarget).minus(Rotation2d.fromDegrees(4.0)));
                    retreat = Poses.HANDOFF_ELEV_UP; // your original behavior
                    uperbody.setTargetPose(retreat);
                    scorePhase = ScorePhase.RETREAT;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case RETREAT -> {
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS) {
                    wanted = WantedSuperState.HANDOFF_CORAL_TO_ARM;
                    scoreNowLatched = false;
                    scorePhase = ScorePhase.TO_HANDOFF;
                }
            }
            case TO_HANDOFF -> {
                /* handoff runner will take over */
            }
        }
    }

    // NEW: Barge shoot (auto stow after)
    private static final double kBargeShootHoldS = 0.40;

    private void runBargeShootPipeline() {
        double t = Timer.getFPGATimestamp() - phaseStart;

        switch (bargePhase) {
            case MOVE -> {
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS) {
                    arm.rollerOutFast();
                    bargePhase = BargePhase.SHOOT;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case SHOOT -> {
                if (t >= kBargeShootHoldS) {
                    arm.rollerStop();
                    wanted = WantedSuperState.DEFAULT_STATE; // auto-stow
                    bargePhase = BargePhase.TO_STOW;
                }
            }
            case TO_STOW -> {
                /* stow machine will take over */
            }
        }
    }

    // NEW: Algae intake (auto stow after)
    private static final double kAlgaeIntakeHoldS = 0.40;

    private void runAlgaeIntakePipeline() {
        double t = Timer.getFPGATimestamp() - phaseStart;

        switch (algaePhase) {
            case MOVE -> {
                if (uperbody.reachedSetpoint() && t >= kPoseSettleS) {
                    arm.rollerIn();
                    algaePhase = AlgaePhase.INTAKE_HOLD;
                    phaseStart = Timer.getFPGATimestamp();
                }
            }
            case INTAKE_HOLD -> {
                if (t >= kAlgaeIntakeHoldS && Algae.getBoolean(false)) {
                    arm.rollerStop();
                    wanted = WantedSuperState.DEFAULT_STATE; // auto-stow after grabbing
                    algaePhase = AlgaePhase.TO_STOW;
                }
            }
            case TO_STOW -> {
                /* stow machine will take over */
            }
        }
    }

    // ───────── Helpers ─────────
    private UpperBoddy.Pose maybeOffsets(UpperBoddy.Pose base) {
        if (!manualModeEntry.getBoolean(false)) return base;
        double delev = manualElevDeltaEntry.getDouble(0.0);
        double ddeg = manualShoulderDeltaDegEntry.getDouble(0.0);
        return Poses.pose(Poses.elevM(base) + delev, Poses.shoulderR(base).plus(Rotation2d.fromDegrees(ddeg)));
    }

    private boolean isPivotUp() {
        return pivotUpManualEntry.getBoolean(false);
    }

    private WantedSuperState getChosenScore() {
        WantedSuperState chosen = scoreChooser.getSelected();
        return (chosen != null) ? chosen : WantedSuperState.SCORE_L2_LEFT;
    }

    private UpperBoddy.Pose poseFromChooser() {
        WantedSuperState chosen = getChosenScore();
        SmartDashboard.putString("SS/ChosenScore", chosen.name());
        Pose base =
                switch (chosen) {
                    case SCORE_L1_LEFT_HIGH -> Poses.withLeft(Poses.L1_HIGH_BASE);
                    case SCORE_L1_LEFT_LOW -> Poses.withLeft(Poses.L1_LOW_BASE);
                    case SCORE_L1_RIGHT_HIGH -> Poses.withRight(Poses.L1_HIGH_BASE);
                    case SCORE_L1_RIGHT_LOW -> Poses.withRight(Poses.L1_LOW_BASE);
                    case SCORE_L2_LEFT -> Poses.withLeft(Poses.L2_BASE);
                    case SCORE_L2_RIGHT -> Poses.withRight(Poses.L2_BASE);
                    case SCORE_L3_LEFT -> Poses.withLeft(Poses.L3_BASE);
                    case SCORE_L3_RIGHT -> Poses.withRight(Poses.L3_BASE);

                        // Algae poses
                    case INTAKE_ALGAE_L2 -> Poses.ALGAE_L2_INTAKE;
                    case INTAKE_ALGAE_L3 -> Poses.ALGAE_L3_INTAKE;

                        // 🔹 new: barge selectable
                    case BARGE_SHOOT -> Poses.BARGE;

                    default -> Poses.withLeft(Poses.L2_BASE);
                };
        return maybeOffsets(base);
    }

    public enum PoseLabel {
        STOW_FINAL,
        STOW_ARM_45,
        INTAKE,
        HANDOFF_ELEV_UP,
        HANDOFF_ARM_ZERO,
        HANDOFF_DROP,
        L1_HIGH_LEFT,
        L1_HIGH_RIGHT,
        L1_LOW_LEFT,
        L1_LOW_RIGHT,
        L2_LEFT,
        L2_RIGHT,
        L3_LEFT,
        L3_RIGHT,
        BARGE,
        ALGAE_L2,
        ALGAE_L3,
        UNKNOWN
    }

    private record PosePoint(double elevM, double shoulderDeg) {}

    /** Weight shoulder angle so meters vs degrees compare sensibly. Tune if needed. */
    private final double kShoulderDegWeight = 0.01; // "meters per degree" weight

    private PosePoint toPoint(UpperBoddy.Pose p) {
        return new PosePoint(p.elev().in(Meters), p.shoulder().in(Degrees));
    }

    private double poseDistance(PosePoint a, PosePoint b) {
        double de = a.elevM() - b.elevM();
        double d0 = (a.shoulderDeg() - b.shoulderDeg()) * kShoulderDegWeight;
        return Math.hypot(de, d0);
    }

    /** Return the best-matching label for the *current* measured pose. */
    public PoseLabel getCurrentPoseLabel() {
        // Prefer measured; fall back to target if sensors not ready
        UpperBoddy.Pose p = uperbody.getMeasuredPose();
        if (p == null) p = uperbody.getTargetPose();
        if (p == null) return PoseLabel.UNKNOWN;

        PosePoint cur = toPoint(p);

        // Candidate set (expand/trim as you like)
        Map<PoseLabel, UpperBoddy.Pose> refs = new LinkedHashMap<>();
        refs.put(PoseLabel.STOW_FINAL, Poses.STOW_FINAL);
        refs.put(PoseLabel.STOW_ARM_45, Poses.STOW_ARM_45);
        refs.put(PoseLabel.INTAKE, Poses.INTAKE);
        refs.put(PoseLabel.HANDOFF_ELEV_UP, Poses.HANDOFF_ELEV_UP);
        refs.put(PoseLabel.HANDOFF_ARM_ZERO, Poses.HANDOFF_ARM_ZERO);
        refs.put(PoseLabel.HANDOFF_DROP, Poses.HANDOFF_DROP);

        refs.put(PoseLabel.L1_HIGH_LEFT, Poses.withLeft(Poses.L1_HIGH_BASE));
        refs.put(PoseLabel.L1_HIGH_RIGHT, Poses.withRight(Poses.L1_HIGH_BASE));
        refs.put(PoseLabel.L1_LOW_LEFT, Poses.withLeft(Poses.L1_LOW_BASE));
        refs.put(PoseLabel.L1_LOW_RIGHT, Poses.withRight(Poses.L1_LOW_BASE));
        refs.put(PoseLabel.L2_LEFT, Poses.withLeft(Poses.L2_BASE));
        refs.put(PoseLabel.L2_RIGHT, Poses.withRight(Poses.L2_BASE));
        refs.put(PoseLabel.L3_LEFT, Poses.withLeft(Poses.L3_BASE));
        refs.put(PoseLabel.L3_RIGHT, Poses.withRight(Poses.L3_BASE));

        refs.put(PoseLabel.BARGE, Poses.BARGE);
        refs.put(PoseLabel.ALGAE_L2, Poses.ALGAE_L2_INTAKE);
        refs.put(PoseLabel.ALGAE_L3, Poses.ALGAE_L3_INTAKE);

        // Find nearest reference
        PoseLabel best = PoseLabel.UNKNOWN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (var e : refs.entrySet()) {
            double d = poseDistance(cur, toPoint(e.getValue()));
            if (d < bestDist) {
                bestDist = d;
                best = e.getKey();
            }
        }

        // Optional gating: if we're far from *any* known pose, call it UNKNOWN
        // Tune this threshold to your robot; start around 0.12–0.20.
        double kMinConfidenceDist = 0.15;
        return (bestDist <= kMinConfidenceDist) ? best : PoseLabel.UNKNOWN;
    }

    /** Convenience: collapse to coarse level (L1/L2/L3/Intake/Stow/etc.). */
    public String getCurrentLevelSimple() {
        switch (getChosenScore()) {
            case SCORE_L1_LEFT_HIGH, SCORE_L1_RIGHT_LOW, SCORE_L1_RIGHT_HIGH, SCORE_L1_LEFT_LOW -> {
                return "L1";
            }
            case SCORE_L2_LEFT, SCORE_L2_RIGHT -> {
                return "L2";
            }
            case SCORE_L3_LEFT, SCORE_L3_RIGHT -> {
                return "L3";
            }
            case INTAKE_CORAL -> {
                return "INTAKE";
            }
            case HANDOFF_CORAL_TO_ARM -> {
                return "HANDOFF";
            }
            case DEFAULT_STATE -> {
                return "STOW";
            }
            case BARGE_SHOOT -> {
                return "BARGE";
            }
            case INTAKE_ALGAE_L2 -> {
                return "ALGAE L2";
            }
            case INTAKE_ALGAE_L3 -> {
                return "ALGAE L3";
            }
            default -> {
                return "UNKNOWN";
            }
        }
    }
}
