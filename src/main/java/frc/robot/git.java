package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class git {
    public enum CoralLocation {
        NONE,
        IN_INTAKE,
        IN_ARM
    }

    public static class gitoutput {
        public final Transform3d elevatorT;
        public final Transform3d armT;
        public final Transform3d carriageT;
        public final Transform3d coralT;

        public gitoutput(Transform3d elevatorT, Transform3d armT, Transform3d carriageT, Transform3d coralT) {
            this.elevatorT = elevatorT;
            this.armT = armT;
            this.carriageT = carriageT;
            this.coralT = coralT;
        }

        public Pose3d getElevatorPose() {
            return Pose3d.kZero.transformBy(elevatorT);
        }

        public Pose3d getCarriagePose() {
            return Pose3d.kZero.transformBy(carriageT);
        }

        public Pose3d getArmPose() {
            return Pose3d.kZero.transformBy(armT);
        }

        public Pose3d getCoralPose() {
            return Pose3d.kZero.transformBy(coralT);
        }
    }

    public static final Transform3d elevatorCarriageToArmPivot =
            new Transform3d(0, 0, Units.inchesToMeters(0), Rotation3d.kZero);

    // intakeParam is now pivot angle in degrees
    public static final gitoutput computate(
            double intakePivotDeg,
            double stage1,
            double carriage,
            double armDeg,
            String score,
            CoralLocation coralLocation) {

        // Elevator and arm transforms (same idea as before)
        Transform3d robottoElevatorTransform = new Transform3d(
                        Units.inchesToMeters(0), 0.0, Units.inchesToMeters(0), Rotation3d.kZero)
                .plus(new Transform3d(0, 0, stage1, Rotation3d.kZero));

        Transform3d robottoElevatorTransformcarriahge = new Transform3d(
                        Units.inchesToMeters(0), 0.0, Units.inchesToMeters(0), Rotation3d.kZero)
                .plus(new Transform3d(0, 0, carriage, Rotation3d.kZero));

        Transform3d robotToArmTransform = robottoElevatorTransformcarriahge.plus(new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-3), Units.inchesToMeters(0)),
                new Rotation3d(0, Units.degreesToRadians(armDeg), 0)));

        // Coral relative to arm (same as you had before)
        Transform3d gripperToCoral = new Transform3d(
                Units.inchesToMeters(0), Units.inchesToMeters(-5.876), Units.inchesToMeters(20.688), Rotation3d.kZero);

        Transform3d coralT;

        switch (coralLocation) {
            case IN_INTAKE -> {
                double pivotRad = Units.degreesToRadians(intakePivotDeg);

                // Tune these three numbers to match your ground intake geometry
                Transform3d robotToIntakePivot = new Transform3d(
                        Units.inchesToMeters(10.0), // forward from robot center to intake pivot
                        Units.inchesToMeters(0.0), // sideways
                        Units.inchesToMeters(6.0), // height off floor
                        Rotation3d.kZero);

                Transform3d pivotRotation = new Transform3d(new Translation3d(), new Rotation3d(0.0, pivotRad, 0.0));

                Transform3d pivotToCoral = new Transform3d(
                        Units.inchesToMeters(10.0), // length from pivot to coral
                        0.0,
                        0.0,
                        Rotation3d.kZero);

                coralT = robotToIntakePivot.plus(pivotRotation).plus(pivotToCoral);
            }
            case IN_ARM -> {
                coralT = robotToArmTransform.plus(gripperToCoral);
            }
            default -> {
                // No coral owned: identity transform (robot origin)
                coralT = new Transform3d();
            }
        }

        return new gitoutput(robottoElevatorTransform, robotToArmTransform, robottoElevatorTransformcarriahge, coralT);
    }
}
