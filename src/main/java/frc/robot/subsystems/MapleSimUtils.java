package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.git;
import frc.robot.git.gitoutput;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class MapleSimUtils {
    /** Simulates ejecting the coral from the gripper */
    public static void scoreCoral(Drive swerve, UpperBoddy upperBoddy, Armwheels arm, String level) {
        final gitoutput kinematicsOutput = git.computate(
                0.0,
                MathUtil.clamp(
                        upperBoddy.getelevatorpos(), 0, MathUtil.clamp(upperBoddy.getelevatorpos() / 2.5, 0.3, 0.6)),
                upperBoddy.getelevatorpos(),
                upperBoddy.getpos(),
                "SCORE",
                git.CoralLocation.IN_ARM);
        final Pose3d coralPose = kinematicsOutput.getCoralPose();
        double degree = 0;
        double height = 0;

        if (level == "L3") {
            degree = -35;
            height = 1.28;

        } else if (level == "L2") {
            degree = -35;
            height = 0.9;
        }

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        swerve.getPose().getTranslation(),
                        // The scoring mechanism releases the coral at this position on the
                        // robot
                        new Translation2d(0.4, -0.2),
                        // Obtain robot speed from drive simulation
                        swerve.getChassisSpeeds(),
                        // Obtain robot facing from drive simulation
                        swerve.getPose().getRotation(),
                        // The height at which the coral is ejected
                        Meters.of(height),

                        // The initial speed of the coral
                        MetersPerSecond.of(1),
                        // Angle coral is ejected at
                        Degrees.of(degree)));
    }

    //   public static void scoreAlgae(
    //       Drive swerve,
    //       UpperBoddy upperBoddy,
    //       Armwheels arm
    //       ) {
    //     final ScoringKinematicsOutput kinematicsOutput =
    //         ScoringKinematics.computeForwardKinematics(
    //             elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    //     final Pose3d algaePose = kinematicsOutput.getAlgaePose();
    //     final boolean isFlipped = algaePose.getRotation().getX() > 0.5 * Math.PI;
    //     SimulatedArena.getInstance()
    //         .addGamePieceProjectile(
    //             new ReefscapeAlgaeOnFly(
    //                 // Obtain robot position from drive simulation
    //                 swerve.getPose().getTranslation(),
    //                 // The scoring mechanism releases the algae at this position on the
    //                 // robot
    //                 algaePose.toPose2d().getTranslation(),
    //                 // Obtain robot speed from drive simulation
    //                 swerve.getchassisSpeeds(),
    //                 // Obtain robot facing from drive simulation
    //                 swerve
    //                     .getPose()
    //                     .getRotation()
    //                     .plus(isFlipped ? Rotation2d.k180deg : Rotation2d.kZero),
    //                 // The height at which the algae is ejected
    //                 algaePose.getMeasureZ(),
    //                 // The initial speed of the algae
    //                 MetersPerSecond.of(3),
    //                 // Angle algae is ejected at
    //                 algaePose.getRotation().getMeasureY().times(-1.0)));
    //   }

    //   public static void scoreAlgaeInNet(
    //       Drive swerve,
    //       UpperBoddy upperBoddy,
    //       Armwheels arm
    //       ) {
    //     final ScoringKinematicsOutput kinematicsOutput =
    //         ScoringKinematics.computeForwardKinematics(
    //             elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    //     final Pose3d algaePose = kinematicsOutput.getAlgaePose();
    //     SimulatedArena.getInstance()
    //         .addGamePieceProjectile(
    //             new ReefscapeAlgaeOnFly(
    //                 // Obtain robot position from drive simulation
    //                 swerve.getPose().getTranslation(),
    //                 // The scoring mechanism releases the algae at this position on the
    //                 // robot
    //                 algaePose.toPose2d().rotateBy(Rotation2d.k180deg).getTranslation(),
    //                 // Obtain robot speed from drive simulation
    //                 swerve.getFieldRelativeChassisSpeeds(),
    //                 // Obtain robot facing from drive simulation
    //                 swerve.getPose().getRotation().plus(Rotation2d.k180deg),
    //                 // The height at which the algae is ejected
    //                 algaePose.getMeasureZ(),
    //                 // The initial speed of the algae
    //                 MetersPerSecond.of(5),
    //                 // Angle algae is ejected at
    //                 new Rotation3d(0, 0.25 * Math.PI, 0).getMeasureY()));
    //   }
    // }
}
