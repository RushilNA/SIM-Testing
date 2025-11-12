package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Intake pivot SIM using WPILib SingleJointedArmSim. - 1 motor, 150:1 gear ratio - Simple internal PD loop for
 * setPosition(), or open-loop percent via setPercent() - Angles are relative to zeroHere()
 */
public class IntakeIOSIM implements IntakeIO {

    // ───────── Config (tweak as needed) ─────────
    private static final double kGearRatio = 150.0; // motor revs : pivot rev
    private static final DCMotor kMotor = DCMotor.getFalcon500(1); // swap to Kraken if you prefer
    private static final double kArmLength = 0.30; // meters (effective COM radius)
    private static final double kArmMassKg = 2.0; // kg (approx for intake + bracket)
    // Rough MOI estimate for a slender rod pivoting about one end: (1/3)*m*L^2
    private static final double kMOI = (1.0 / 3.0) * kArmMassKg * kArmLength * kArmLength;

    // Motion limits (radians). Example: -20° to +120°
    private static final double kMinAngleRad = Units.degreesToRadians(-20.0);
    private static final double kMaxAngleRad = Units.degreesToRadians(+120.0);

    // Control (very basic PD for sim position control)
    private static final double kP_VoltPerRad = 18.0; // proportional term → volts/rad
    private static final double kD_VoltPerRadPerS = 1.0; // derivative term → volts/(rad/s)

    // Sim step
    private static final double kDt = 0.02; // 20 ms

    // ───────── Internal State ─────────
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            kMotor, // motor model
            kGearRatio, // gear reduction
            kMOI, // moment of inertia about joint
            kArmLength, // arm length to COM
            kMinAngleRad, // min angle (rad)
            kMaxAngleRad, // max angle (rad)
            true, // simulate gravity
            0.0 // starting angle (rad)
            );

    private enum Mode {
        PERCENT,
        POSITION
    }

    private Mode mode = Mode.PERCENT;

    // open-loop percent ([-1, +1]) and closed-loop setpoint (rad)
    private double percentCmd = 0.0;
    private double targetRad = 0.0;

    // zero offset handling (sim angle – zeroOffset = reported angle)
    private double zeroOffsetRad = 0.0;

    // last commanded voltage (for logging)
    private double lastVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Decide control action
        double volts;
        if (mode == Mode.PERCENT) {
            volts = MathUtil.clamp(percentCmd * 12.0, -12.0, 12.0);
        } else {
            // PD control on (target - actual)
            double angleRad = sim.getAngleRads();
            double velRadPerS = sim.getVelocityRadPerSec();
            double error = targetRad - angleRad;
            volts = kP_VoltPerRad * error - kD_VoltPerRadPerS * velRadPerS;
            volts = MathUtil.clamp(volts, -12.0, 12.0);
        }

        // Feed the sim and advance
        sim.setInputVoltage(volts);
        sim.update(kDt);
        lastVolts = volts;

        // Populate inputs (convert to user-facing units; apply zero offset)
        double angleWithZero = sim.getAngleRads() - zeroOffsetRad;
        double velWithZero = sim.getVelocityRadPerSec(); // zero offset doesn't affect velocity

        inputs.motorConnected = true;
        inputs.busVoltage = Volts.of(RobotController.getBatteryVoltage());
        inputs.appliedVoltage = Volts.of(lastVolts);

        inputs.intakeAngle = Radians.of(angleWithZero);
        inputs.velocity = RotationsPerSecond.of(velWithZero / (2.0 * Math.PI));

        // simple current estimates (SingleJointedArmSim exposes motor current)
        inputs.statorCurrent = edu.wpi.first.units.Units.Amps.of(sim.getCurrentDrawAmps());
        inputs.supplyCurrent = inputs.statorCurrent; // close enough for sim
    }

    @Override
    public void setPercent(double pct) {
        mode = Mode.PERCENT;
        percentCmd = MathUtil.clamp(pct, -1.0, 1.0);
    }

    @Override
    public void setPosition(Angle angle) {
        mode = Mode.POSITION;
        // Angle is relative to zeroHere(); convert to absolute sim angle
        double desiredRad = angle.in(Radians) + zeroOffsetRad;
        // Respect physical limits to avoid integral windup/banging into stops
        desiredRad = MathUtil.clamp(desiredRad, kMinAngleRad, kMaxAngleRad);
        targetRad = desiredRad;
    }

    @Override
    public void zeroHere() {
        // Make the current sim angle read as 0
        zeroOffsetRad = sim.getAngleRads();
    }

    @Override
    public void stop() {
        mode = Mode.PERCENT;
        percentCmd = 0.0;
    }
}
