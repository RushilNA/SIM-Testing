package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Visualization for the arm subsystem in simulation. */
public class Realarmsim extends SubsystemBase {

    private final Realarm arm;

    // Simulation display
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d armMech;

    // Visualization constants (pixels-ish)
    private static final double BASE_WIDTH = 40.0;
    private static final double BASE_HEIGHT = 20.0;
    private static final double TOWER_HEIGHT = 30.0;
    private static final double ARM_WIDTH = 10.0;

    // Arm parameters
    private final double armLength;
    private final double visualScaleFactor;

    /**
     * Creates a new visualization for the arm.
     *
     * @param armSubsystem The arm subsystem to visualize
     */
    public Realarmsim(Realarm armSubsystem) {
        this.arm = armSubsystem;

        // Get arm length from subsystem (in meters)
        armLength = 1.0;

        // Scale so the arm appears ~200 px long
        visualScaleFactor = 200.0 / armLength;

        // Create the simulation display
        mech = new Mechanism2d(400, 400);
        root = mech.getRoot("ArmRoot", 200, 200);

        // Base
        MechanismLigament2d armBase = root.append(
                new MechanismLigament2d("Base", BASE_WIDTH, 0.0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

        // Tower
        MechanismLigament2d tower = armBase.append(
                new MechanismLigament2d("Tower", TOWER_HEIGHT, 90.0, BASE_HEIGHT / 2.0, new Color8Bit(Color.kGray)));

        // Pivot
        MechanismLigament2d pivot =
                tower.append(new MechanismLigament2d("Pivot", 5.0, 0.0, 5.0, new Color8Bit(Color.kBlack)));

        // Arm stick
        armMech = pivot.append(new MechanismLigament2d(
                "Arm", armLength * visualScaleFactor, 0.0, ARM_WIDTH, new Color8Bit(Color.kBlue)));

        // Initialize visualization
        SmartDashboard.putData("Arm Sim", mech);
    }

    @Override
    public void periodic() {
        // Update arm angle from sim (mechanism frame, radians)
        double currentAngleRad = arm.getSimulation().getAngleRads();
        armMech.setAngle(Units.radiansToDegrees(currentAngleRad));

        // Telemetry
        SmartDashboard.putNumber("Arm Angle (deg)", Units.radiansToDegrees(currentAngleRad));
        SmartDashboard.putNumber(
                "Arm Velocity (deg/s)",
                Units.radiansToDegrees(arm.getSimulation().getVelocityRadPerSec()));
        SmartDashboard.putNumber("Arm Current (A)", arm.getSimulation().getCurrentDrawAmps());
    }
}
