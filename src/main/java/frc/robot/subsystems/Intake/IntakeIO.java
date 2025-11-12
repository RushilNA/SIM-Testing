package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean motorConnected = false;

        public Voltage busVoltage = Volts.of(0);
        public Voltage appliedVoltage = Volts.of(0);

        public Current statorCurrent = Amps.of(0);
        public Current supplyCurrent = Amps.of(0);

        /** Pivot angle relative to last zeroHere(). */
        public Angle intakeAngle = Degrees.of(0);

        /** Angular velocity from integrated sensor. */
        public AngularVelocity velocity = RotationsPerSecond.of(0);
    }

    /** Update loggable inputs. */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Open-loop control in [-1, +1] (percent output). */
    default void setPercent(double pct) {}

    /** Closed-loop position (relative to zeroHere). */
    default void setPosition(Angle angle) {}

    /** Mark the current integrated sensor position as zero. */
    default void zeroHere() {}

    /** Stop motor output. */
    default void stop() {}
}
