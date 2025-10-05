package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm subsystem using TalonFX with Kraken X60 motor. */
@Logged(name = "RealArm")
public class Realarm extends SubsystemBase {

    // ===== Constants =====
    private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
    private final int canID = 1;

    /**
     * Motor/sensor rotations per mechanism rotation (gear reduction). Must be > 1 for a typical arm. Replace with your
     * real reduction.
     */
    private final double gearRatio = 60.0;

    // CTRE Slot0 PID/FF (used by Position/Velocity/MotionMagic requests)
    private final double kP = 10.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kS = 0.3;
    private final double kV = 0.0;
    private final double kA = 0.0;
    private final double kG_ctre = 0.15;

    // Motion Magic constraints (mechanism units: rotations/sec and rotations/sec^2)
    private final double mmCruiseRps = 0.5; // ~180 deg/s
    private final double mmAccelRps2 = 1.0; // reasonable ramp

    // For the simple velocity-profile command
    private final double maxVelocity = 30.0; // rad/s
    private final double maxAcceleration = 1.0; // rad/s^2

    private final boolean brakeMode = true;

    // Current limits
    private final boolean enableStatorLimit = true;
    private final double statorCurrentLimit = 80.0;
    private final boolean enableSupplyLimit = true;
    private final double supplyCurrentLimit = 80.0;

    // Arm geometry (for physics sim)
    private final double armLength = 1.0; // meters
    private final double armMassKg = 2.721552; // kg

    // ===== Motor controller =====
    private final TalonFX motor;

    // Control requests
    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;
    private final MotionMagicVoltage motionMagicRequest;

    // Telemetry signals
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    // ===== Simulation =====
    private final SingleJointedArmSim armSim;

    /** Creates a new Arm Subsystem. */
    public Realarm() {
        // Initialize motor controller
        motor = new TalonFX(canID);

        // Create control requests (Slot 0)
        positionRequest = new PositionVoltage(0).withSlot(0);
        velocityRequest = new VelocityVoltage(0).withSlot(0);
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

        // Status signals
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        temperatureSignal = motor.getDeviceTemp();

        // === Talon configuration ===
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID/FF gains (Slot 0)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kG = kG_ctre;

        // Motion Magic constraints (mechanism units thanks to SensorToMechanismRatio)
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = mmCruiseRps; // rotations/sec
        mm.MotionMagicAcceleration = mmAccelRps2; // rotations/sec^2
        // Optional if supported: mm.MotionMagicJerk = ...

        // Current limits
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.StatorCurrentLimit = statorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
        currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

        // Neutral mode
        config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        // Feedback ratio: sensor rotations per mechanism rotation
        config.Feedback.SensorToMechanismRatio = gearRatio;

        // Apply configuration
        motor.getConfigurator().apply(config);

        // Zero encoder
        motor.setPosition(0);

        // === Physics sim ===
        armSim = new SingleJointedArmSim(
                dcMotor,
                gearRatio, // reduction (>1)
                SingleJointedArmSim.estimateMOI(armLength, armMassKg),
                armLength,
                0.0, // min angle (rad)
                Math.PI / 2.0, // max angle (rad)
                true, // simulate gravity
                0.0 // starting angle (rad)
                );

        // Provide a supply voltage in sim
        motor.getSimState().setSupplyVoltage(12.0);
    }

    /** Update telemetry. */
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
    }

    /** Update simulation. */
    @Override
    public void simulationPeriodic() {
        // 1) Use the voltage the Talon is actually applying
        double appliedVolts = motor.getSimState().getMotorVoltage();

        // 2) Drive the physics model
        armSim.setInput(appliedVolts);

        // 3) Step physics (20 ms)
        armSim.update(0.020);

        // 4) Sim battery sag
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // 5) Mechanism -> rotor conversion using gear reduction
        double mechRots = armSim.getAngleRads() / (2.0 * Math.PI);
        double mechRps = armSim.getVelocityRadPerSec() / (2.0 * Math.PI);

        double rotorRots = mechRots * gearRatio;
        double rotorRps = mechRps * gearRatio;

        motor.getSimState().setRawRotorPosition(rotorRots); // rotations
        motor.getSimState().setRotorVelocity(rotorRps); // rotations per second
    }

    // ===== Readbacks =====

    /** @return Position in mechanism rotations (because SensorToMechanismRatio is set) */
    @Logged(name = "Position/Rotations")
    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    /** @return Velocity in mechanism rotations per second */
    @Logged(name = "Velocity")
    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    /** @return Applied voltage */
    @Logged(name = "Voltage")
    public double getVoltage() {
        return voltageSignal.getValueAsDouble();
    }

    /** @return Motor stator current in amps */
    @Logged(name = "Current")
    public double getCurrent() {
        return statorCurrentSignal.getValueAsDouble();
    }

    /** @return Motor temperature in °C */
    @Logged(name = "Temperature")
    public double getTemperature() {
        return temperatureSignal.getValueAsDouble();
    }

    public double getPositionRadians() {
        return Units.rotationsToRadians(getPosition());
    }

    public double getVelocityRadPerSec() {
        return Units.rotationsToRadians(getVelocity());
    }

    // ===== Controls =====

    /** Set arm to an absolute angle in degrees using CTRE Position PID (no profiling). */
    public void setAngle(double angleDegrees) {
        setAngle(angleDegrees, 0.0);
    }

    /**
     * Set arm angle with (optional) acceleration hint for FF (still Position PID).
     *
     * @param angleDegrees target angle in degrees (mechanism frame)
     * @param accelerationRadPerSec2 acceleration in rad/s^2 (currently unused by CTRE FF unless kA set)
     */
    public void setAngle(double angleDegrees, double accelerationRadPerSec2) {
        double angleRad = Units.degreesToRadians(angleDegrees);
        double positionRotations = angleRad / (2.0 * Math.PI); // mechanism rotations
        motor.setControl(positionRequest.withPosition(positionRotations));
        // To add FF volts from RIO-side, you could do: .withFeedForward(ffVolts)
    }

    /** Set arm to an absolute angle in degrees using Motion Magic (profiles + holds). */
    public void setAngleMM(double angleDegrees) {
        double angleRad = Units.degreesToRadians(angleDegrees);
        double positionRotations = angleRad / (2.0 * Math.PI); // mechanism rotations
        motor.setControl(motionMagicRequest.withPosition(positionRotations));
    }

    /** Set velocity in deg/s using CTRE Velocity PID (no profiling). */
    public void setVelocity(double velocityDegPerSec) {
        setVelocity(velocityDegPerSec, 0.0);
    }

    /** Velocity setpoint with optional accel (deg/s^2) for FF (if kA used). */
    public void setVelocity(double velocityDegPerSec, double accelerationDegPerSec2) {
        double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
        double velocityRotationsPerSec = velocityRadPerSec / (2.0 * Math.PI); // mechanism rotations/sec
        motor.setControl(velocityRequest.withVelocity(velocityRotationsPerSec));
        // To add FF volts from RIO-side, you could do: .withFeedForward(ffVolts)
    }

    /** Direct voltage (open loop). */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /** Expose the arm sim for the visualizer. */
    public SingleJointedArmSim getSimulation() {
        return armSim;
    }

    // ===== Commands =====

    /** Position PID, simple velocity profile approach, then ends (no hold). */
    public Command moveToAngleCommand(double angleDegrees) {
        return run(() -> {
                    double currentAngleDeg = Units.radiansToDegrees(getPositionRadians());
                    double error = angleDegrees - currentAngleDeg;

                    double maxVelDegPerSec = Units.radiansToDegrees(maxVelocity);
                    double commandedVelDegPerSec =
                            Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelDegPerSec);

                    setVelocity(commandedVelDegPerSec); // velocity loop (Slot0)
                })
                .until(() -> {
                    double currentAngleDeg = Units.radiansToDegrees(getPositionRadians());
                    return Math.abs(angleDegrees - currentAngleDeg) < 2.0; // 2-degree tolerance
                })
                .finallyDo(interrupted -> setAngle(angleDegrees)); // switch to position hold at target
    }

    /** One-shot: set the angle (Position PID) and hold. */
    public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setAngle(angleDegrees));
    }

    /** Motion Magic version: profile to the angle and hold while scheduled. */
    public Command moveToAngleMMCommand(double angleDegrees) {
        return run(() -> setAngleMM(angleDegrees));
        // For a fire-and-forget style: return runOnce(() -> setAngleMM(angleDegrees));
    }

    /** Stop: requests zero velocity (does not hold position). */
    public Command stopCommand() {
        return runOnce(() -> setVelocity(0.0));
    }

    /** Command to run at a constant velocity (deg/s). */
    public Command moveAtVelocityCommand(double velocityDegPerSec) {
        return run(() -> setVelocity(velocityDegPerSec));
    }
}
