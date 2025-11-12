package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Roller subsystem mounted on the arm for scoring/intake. */
public class Armwheels extends SubsystemBase {

    // Hardware + sim models
    private final TalonFX rollerFx = new TalonFX(223);
    private final TalonFXSimState leaderSim;
    private final DCMotor motor = DCMotor.getKrakenX60Foc(1);

    // Plant for DCMotorSim (kept from your original)
    private final DCMotorSim motorSimModel;

    // NOTE: kept your spelling to avoid breaking references elsewhere
    public double vlotage;

    // Requested state from the rest of the robot
    public enum wantedState {
        OFF,
        SLOW,
        FAST,
        IN
    }

    // The state we are actually applying right now
    public enum curentState {
        OFF,
        SLOW,
        FAST,
        IN
    }

    private wantedState wState = wantedState.OFF;
    private curentState cState = curentState.OFF;

    // --- (Optional) extra sim-only tracking for dashboards ---
    private double simVelocity = 0.0; // arbitrary display units (kept for your SmartDashboard)
    private static final double SIM_KV = 500.0; // voltage → velocity scale (unused but preserved)
    private static final double SIM_ALPHA = 0.12; // response (unused but preserved)

    public Armwheels() {
        leaderSim = rollerFx.getSimState();

        // Estimate MOI from a 1.5" radius, 8 lb wheel assembly (your original math)
        Distance radius = Inches.of(1.5);
        double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);

        // Create a simple DC motor system model for sim
        LinearSystem<N2, N1, N2> linearSystem = LinearSystemId.createDCMotorSystem(motor, moi, 1.5);
        motorSimModel = new DCMotorSim(linearSystem, motor);
    }

    // ───────────────────────────────── periodic ─────────────────────────────────
    @Override
    public void periodic() {
        // 1) State machine: decide → apply
        cState = handleStateTransition();
        applyState();

        // 2) Real hardware telemetry (will be ~0 in sim without a sensor model)
        SmartDashboard.putNumber(
                "Roller RotorVel (rad/s)", rollerFx.getRotorVelocity().getValueAsDouble());
        Logger.recordOutput("Armwheels/WantedState", wState);
        Logger.recordOutput("Armwheels/CState", cState);
        Logger.recordOutput("States/armwheelsWantedState", wState);
        Logger.recordOutput("States/armwheelsCState", cState);
        Logger.recordOutput("Armwheels/RollerVel_Sensor", rollerFx.getVelocity().getValueAsDouble());

        // 3) Sim-only visibility
        Logger.recordOutput("Armwheels/RollerVel_Sim", simVelocity);
        SmartDashboard.putNumber("Simulated Roller Velocity", simVelocity);
        SmartDashboard.putNumber("Armwheels Commanded Voltage", vlotage);
    }

    // ───────────────────────── State machine helpers ───────────────────────────
    private curentState handleStateTransition() {
        return switch (wState) {
            case OFF -> curentState.OFF;
            case SLOW -> curentState.SLOW;
            case FAST -> curentState.FAST;
            case IN -> curentState.IN;
        };
    }

    public void applyState() {
        double voltage;
        switch (cState) {
            case FAST:
                voltage = 6.0; // outtake fast (scoring)
                break;
            case IN:
                voltage = 3.0; // intake
                break;
            case SLOW:
                voltage = 3.0; // outtake slow
                break;
            case OFF:
            default:
                voltage = 0.0;
                break;
        }
        this.vlotage = voltage;

        // Command hardware (will be virtual in sim)
        rollerFx.setVoltage(voltage);

        // Update a simple sim-only velocity number for dashboards
        // (Not physically accurate; just for visual feedback.)
        if (voltage == 0.0) {
            simVelocity *= 0.90; // crude decay
        } else {
            double target = (voltage > 0) ? (voltage * 100.0) : (voltage * 100.0);
            simVelocity += 0.2 * (target - simVelocity);
        }
    }

    // ─────────────────────────── Public API (commands) ─────────────────────────
    public void setWantedState(wantedState state) {
        this.wState = state;
    }

    public void rollerIn() {
        this.wState = wantedState.IN;
    }

    public void rollerOutSlow() {
        this.wState = wantedState.SLOW;
    }

    public void rollerOutFast() {
        this.wState = wantedState.FAST;
    }

    public void rollerStop() {
        this.wState = wantedState.OFF;
    }

    // ───────────────────── Visual/logic helpers for Superstructure ─────────────
    /** True when the arm rollers are outtaking at high speed (scoring). */
    public boolean isOuttakingFast() {
        return cState == curentState.FAST;
    }

    /** True when the arm rollers are outtaking slowly. */
    public boolean isOuttakingSlow() {
        return cState == curentState.SLOW;
    }

    /** True when the arm rollers are pulling a game piece in. */
    public boolean isIntaking() {
        return cState == curentState.IN;
    }

    /** Optional: expose the commanded voltage for plots/logic. */
    public double getRollerVoltage() {
        return vlotage;
    }

    // ───────────────────────────── Sim update hook ─────────────────────────────
    @Override
    public void simulationPeriodic() {
        // Provide battery voltage to the sim side of the Talon
        leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // If motor voltage is zero, back-compute a hold voltage from the sim to avoid abrupt stops
        double motorVoltage = leaderSim.getMotorVoltage();
        if (motorVoltage == 0) {
            motorVoltage =
                    motorSimModel.getAngularVelocity().times(1.5).in(RadiansPerSecond) / motor.KvRadPerSecPerVolt;
            // simple friction model
            if (motorVoltage > 0.2) {
                motorVoltage -= 0.2;
            } else if (motorVoltage < -0.2) {
                motorVoltage += 0.2;
            } else {
                motorVoltage = 0.0;
            }
        }

        // Drive the plant model, assuming 20 ms loop
        motorSimModel.setInputVoltage(motorVoltage);
        motorSimModel.update(0.020);

        // Push rotor (pre-gear) position/velocity into TalonFX sim state
        // (Your code multiplies by 1.5 as a stand-in gear ratio)
        leaderSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(1.5));
        leaderSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(1.5));

        Logger.recordOutput("Wheel", leaderSim.getMotorVoltage());
    }
}
