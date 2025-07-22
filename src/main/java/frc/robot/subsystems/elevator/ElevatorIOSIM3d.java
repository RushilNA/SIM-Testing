package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Simulation implementation of the elevator subsystem with integrated 3D component-pose logging, including zeroed
 * poses.
 */
public class ElevatorIOSIM3d extends ElevatorIOCTRE {
    private static final double CANVAS_WIDTH = 1;
    private static final double CANVAS_HEIGHT = 10;
    private static final double VISIBLE_MAX_HEIGHT_METERS = 6;

    private final ElevatorSim motorSimModel;
    private final TalonFXSimState leaderSim;
    private final TalonFXSimState followerSim;
    private final CANcoderSimState encoderSim;
    private final Drive swerve;

    // 2D mechanism objects
    private final LoggedMechanism2d mech2d;
    private final LoggedMechanismRoot2d root2d;
    private final LoggedMechanismLigament2d carriage2d;

    public ElevatorIOSIM3d(Drive swerveSubsystem) {
        super();
        this.swerve = swerveSubsystem;

        leaderSim = leader.getSimState();
        followerSim = follower.getSimState();
        encoderSim = encoder.getSimState();

        DCMotor motor = DCMotor.getKrakenX60Foc(2);
        LinearSystem<N2, N1, N2> linearSystem = LinearSystemId.createElevatorSystem(
                motor, Pounds.of(10).in(Kilograms), elevatorRadius.in(Meters), GEAR_RATIO);
        motorSimModel = new ElevatorSim(linearSystem, motor, 0, 2, true, 0);

        // 2D setup
        mech2d = new LoggedMechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
        Logger.recordOutput("Visualization/ElevatorMechanism2D", mech2d);
        root2d = mech2d.getRoot("ElevatorRoot2D", CANVAS_WIDTH / 2, 0);
        carriage2d = root2d.append(new LoggedMechanismLigament2d("Carriage2D", 90, 90));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        super.updateInputs(inputs);

        // simulate motor and encoder
        leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        motorSimModel.setInputVoltage(leaderSim.getMotorVoltage());
        motorSimModel.update(0.020);

        Angle position = Conversions.metersToRotations(Meters.of(motorSimModel.getPositionMeters()), 1, elevatorRadius);
        AngularVelocity velocity = Conversions.metersToRotationsVel(
                MetersPerSecond.of(motorSimModel.getVelocityMetersPerSecond()), 1, elevatorRadius);

        leaderSim.setRawRotorPosition(position.times(GEAR_RATIO));
        leaderSim.setRotorVelocity(velocity.times(GEAR_RATIO));
        encoderSim.setRawPosition(position);
        encoderSim.setVelocity(velocity);

        // update 2D viz
        double heightMeters = Math.min(motorSimModel.getPositionMeters(), VISIBLE_MAX_HEIGHT_METERS);
        double pixelsPerMeter = CANVAS_HEIGHT / VISIBLE_MAX_HEIGHT_METERS;
        carriage2d.setLength(heightMeters * pixelsPerMeter);
        Logger.recordOutput("Visualization/ElevatorMechanism2D", mech2d);

        // ---------- 3D Pose Logging ----------
        Pose2d pose2d = swerve.getPose();
        Pose3d robotPose = new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()));

        double elevH = motorSimModel.getPositionMeters();

        // Compute live component poses
        Pose3d rootComponent = robotPose;
        Pose3d stage1Pose = robotPose.plus(new Transform3d(
                new Translation3d(
                        0,
                        0,
                        MathUtil.clamp(
                                        elevH,
                                        Units.inchesToMeters(27.7875 + 23.5375),
                                        Units.inchesToMeters(27.7875 + 23.5375) * 2)
                                - Units.inchesToMeters(27.7875 + 23.5375)),
                new Rotation3d()));
        Pose3d stage2Pose = robotPose.plus(new Transform3d(
                new Translation3d(
                        0, 0, MathUtil.clamp(elevH, Units.inchesToMeters(23.5375), 48) - Units.inchesToMeters(23.5375)),
                new Rotation3d()));
        Pose3d carriagePose3d = robotPose.plus(new Transform3d(new Translation3d(0, 0, elevH), new Rotation3d()));

        // Log live poses
        Logger.recordOutput("ComponentPoses", rootComponent, stage1Pose, stage2Pose, carriagePose3d);

        // ---------- Zeroed Component Poses Logging ----------
        Pose3d zeroPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d());
        Logger.recordOutput("ComponentPosesZero", zeroPose, zeroPose, zeroPose, zeroPose);
    }
}
