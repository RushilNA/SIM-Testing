package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * CTRE IO for Intake pivot. - One TalonFX - Uses built-in integrated sensor - Gear ratio: 150:1 - Includes motor config
 * internally (no external file)
 */
public class IntakeIOCTRE implements IntakeIO {
    private final TalonFX motor = new TalonFX(30);
    private final double gearRatio = 150.0; // 150 motor revs per pivot rev
    private double zeroOffset = 0.0; // stored in motor rotations

    // Control modes
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage posCtrl = new PositionVoltage(0).withEnableFOC(true);

    // Cached signals
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Angle> motorPos;
    private final StatusSignal<AngularVelocity> motorVel;

    public IntakeIOCTRE() {

        // ─────────────── Motor Configuration ───────────────
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // PID gains (tune these)
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 40.0; // proportional gain (example)
        slot0.kI = 0.0;
        slot0.kD = 0.5;
        slot0.kV = 0.0;
        cfg.Slot0 = slot0;

        // current limit
        cfg.CurrentLimits.SupplyCurrentLimit = 90;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        // voltage and neutral settings
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // apply
        motor.getConfigurator().apply(cfg, 0.25);

        // ─────────────── Cached signals ───────────────
        appliedVoltage = motor.getMotorVoltage();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        motorPos = motor.getPosition();
        motorVel = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100, appliedVoltage, statorCurrent, supplyCurrent, motorPos, motorVel);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode code = BaseStatusSignal.refreshAll(appliedVoltage, statorCurrent, supplyCurrent, motorPos, motorVel);

        inputs.motorConnected = code.isOK();
        inputs.appliedVoltage = appliedVoltage.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();

        // Convert integrated motor rotations → pivot rotations
        double motorRot = motorPos.getValue().in(Units.Rotations);
        double pivotRot = (motorRot - zeroOffset) / gearRatio;
        inputs.intakeAngle = Units.Rotations.of(pivotRot);

        inputs.velocity = Units.RotationsPerSecond.of(motorVel.getValue().in(Units.RotationsPerSecond) / gearRatio);
    }

    @Override
    public void setPercent(double pct) {
        motor.setControl(voltageOut.withOutput(12.0 * pct));
    }

    @Override
    public void setPosition(Angle angle) {
        double targetPivotRot = angle.in(Units.Rotations);
        double targetMotorRot = zeroOffset + targetPivotRot * gearRatio;
        motor.setControl(posCtrl.withPosition(Units.Rotations.of(targetMotorRot)));
    }

    @Override
    public void zeroHere() {
        zeroOffset = motorPos.getValue().in(Units.Rotations);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
