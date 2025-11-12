package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class git {
    public static class gitoutput {
        public final Transform3d elevatorT;
        public final Transform3d armT;
        public final Transform3d carriageT;

        public gitoutput(Transform3d elevatorT, Transform3d armT, Transform3d carriageT) {

            this.elevatorT = elevatorT;
            this.armT = armT;
            this.carriageT = carriageT;
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
    }

    public static final Transform3d elevatorCarriageToArmPivot =
            new Transform3d(0, 0, Units.inchesToMeters(0), Rotation3d.kZero);

    public static final gitoutput computate(double intake, double stage1, double carriage, double arm, String score) {
        final Transform3d robottoElevatorTransform = new Transform3d(
                        Units.inchesToMeters(0), 0.0, Units.inchesToMeters(0), Rotation3d.kZero)
                .plus(new Transform3d(0, 0, stage1, Rotation3d.kZero));

        final Transform3d robottoElevatorTransformcarriahge = new Transform3d(
                        Units.inchesToMeters(0), 0.0, Units.inchesToMeters(0), Rotation3d.kZero)
                .plus(new Transform3d(0, 0, carriage, Rotation3d.kZero));

        Transform3d robotToArmTransform = (robottoElevatorTransformcarriahge)
                .plus(new Transform3d(
                        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-3), Units.inchesToMeters(0)),
                        new Rotation3d(0, Units.degreesToRadians(arm), 0)));

        return new gitoutput(robottoElevatorTransform, robotToArmTransform, robottoElevatorTransformcarriahge);
    }
}
