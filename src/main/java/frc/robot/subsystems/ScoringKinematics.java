package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public final class ScoringKinematics {

    /** Fixed robot geometry (tune these values to your CAD) */
    public static final class Viz3d {
        // Robot → where elevator stage1 begins
        public static final Transform3d robotToElevatorCarriage = new Transform3d(
                Units.inchesToMeters(4.0), // X forward
                0.0, // Y
                Units.inchesToMeters(5.75), // Z height of base of elevator
                Rotation3d.kZero);

        // Stage1 carriage → arm pivot mount location
        public static final Transform3d elevatorCarriageToArmPivot =
                new Transform3d(0.0, 0, Units.inchesToMeters(26.0), Rotation3d.kZero);

        // Precomputed base heights
        public static final double stage1BaseZ = robotToElevatorCarriage.getZ();
        public static final double armPivotBaseZ = stage1BaseZ + elevatorCarriageToArmPivot.getZ();

        // If intake must sit above ground slightly
        public static final double intakeBaseZ = 0.0; // tune if needed
    }

    /** Result containing ALL the Pose3d for the entire mechanism */
    public static final class Output {
        public final Pose3d intakePose;
        public final Pose3d stage1Pose;
        public final Pose3d stage2Pose;
        public final Pose3d armPose;

        public Output(Pose3d intake, Pose3d s1, Pose3d s2, Pose3d arm) {
            this.intakePose = intake;
            this.stage1Pose = s1;
            this.stage2Pose = s2;
            this.armPose = arm;
        }
    }

    /**
     * Forward kinematics for your robot.
     *
     * @param s1 Stage 1 extension (meters)
     * @param s2 Stage 2 extension (meters)
     * @param armDeg arm angle in degrees
     * @param intakeDeg intake angle in degrees
     */
    public static Output compute(double s1, double s2, double armDeg, double intakeDeg) {
        // Clamp to no-negative heights
        s1 = Math.max(0.0, s1);
        s2 = Math.max(0.0, s2);

        // Wrap rotation into [-pi, pi]
        double armRad = MathUtil.angleModulus(Units.degreesToRadians(armDeg));
        double intakeRad = MathUtil.angleModulus(Units.degreesToRadians(intakeDeg));

        // Absolute heights for flat-file components
        double zStage1 = s1;
        double zStage2 = s2;
        double zArm = s2;

        // ----- Build Pose3d objects -----

        Pose3d intakePose = new Pose3d(
                new Translation3d(0, intakeDeg * -0.002, intakeDeg * 0.0042), new Rotation3d(intakeRad, 0, 0));

        Pose3d stage1Pose = new Pose3d(new Translation3d(0.0, 0.0, zStage1), new Rotation3d()); // stays upright

        Pose3d stage2Pose = new Pose3d(new Translation3d(0.0, 0.0, zStage2), Rotation3d.kZero); // stays upright

        // Arm rotates about its hinge — choose correct axis later
        Pose3d armBase = stage2Pose.transformBy(
                new Transform3d(ScoringKinematics.Viz3d.elevatorCarriageToArmPivot.getTranslation(), Rotation3d.kZero));

        // 2) Rotate the arm about that pivot (about +Y; flip sign if needed)
        Rotation3d armRot = new Rotation3d(0.0, armDeg, 0.0);
        Pose3d armPose = armBase.transformBy(new Transform3d(Translation3d.kZero, armRot));
        return new Output(intakePose, stage1Pose, stage2Pose, armPose);
    }

    /** Logs Pose3d values for AdvantageKit (use your own prefix) */
    public static void logViz(String prefix, double s1Meters, double s2Meters, double armDeg, double intakeDeg) {

        Output out = compute(s1Meters, s2Meters, armDeg, intakeDeg);

        Logger.recordOutput(prefix + "/intake", out.intakePose);
        Logger.recordOutput(prefix + "/elevatorStage1", out.stage1Pose);
        Logger.recordOutput(prefix + "/elevatorStage2", out.stage2Pose);
        Logger.recordOutput(prefix + "/arm", out.armPose);
    }

    public static double increasingAsAngleGetsSmaller(double angleDeg, double maxAngleDeg, double maxOutput) {
        angleDeg = MathUtil.clamp(angleDeg, 0, maxAngleDeg);
        double proportion = 1.0 - (angleDeg / maxAngleDeg);
        return maxOutput * proportion;
    }

    private ScoringKinematics() {}
}
