package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private final Alert leaderMotorAlert = new Alert("Arm leader motor isn't connected", AlertType.kError);
    private final Alert followerMotorAlert = new Alert("Arm follower motor isn't connected", AlertType.kError);
    private final Alert encoderAlert = new Alert("Arm encoder isn't connected", AlertType.kError);

    private ArmMode desiredMode = ArmMode.INTAKE;
    private Angle targetAngle = ArmMode.INTAKE.targetAngle;

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        leaderMotorAlert.set(!inputs.leaderConnected);
        followerMotorAlert.set(!inputs.followerConnected);
        encoderAlert.set(!inputs.encoderConnected);

        // Mini-superstructure logic
        if (desiredMode == ArmMode.STOP) {
            stop();
        } else{
            setPosition(targetAngle);
        }

        // Dashboard logging
        SmartDashboard.putString("Arm Mode", desiredMode.name());
        SmartDashboard.putNumber("Arm Position (deg)", getPosition().in(Degrees));
        SmartDashboard.putBoolean("Arm At Target", isAtTarget());
    }

    /** Command the motors to move to a given angle */
    private void setPosition(Angle position) {
        io.setPosition(position);
    }

    /** Stop all arm motors immediately */
    private void stop() {
        io.stop();
    }

    /** Get current arm position */
    @AutoLogOutput
    public Angle getPosition() {
        return inputs.encoderPosition;
    }

    /** Set arm to desired mode */
    public void setState(ArmMode mode) {
        if (desiredMode != mode) {
            desiredMode = mode;
            targetAngle = mode.targetAngle;
        }
    }

    /** Check if arm is at target position */
    @AutoLogOutput
    public boolean isAtTarget() {
        if (desiredMode == ArmMode.STOP) return true; // Treat stop as already "at target"
        return getPosition().isNear(targetAngle, desiredMode.angleTolerance);
    }

    /** Get the current goal angle */
    @AutoLogOutput
    public Angle getTargetAngle() {
        return targetAngle;
    }

    /** Possible states for the arm */
    public enum ArmMode {
        STOP(Degrees.of(0)), // Stops motors — does NOT move to 0
        INTAKE(Degrees.of(0)),
        L1(Degrees.of(90)),
        L2(Degrees.of(135)),
        L3(Degrees.of(135)),
        L4(Degrees.of(180));

        public final Angle targetAngle;
        public final Angle angleTolerance;

        ArmMode(Angle targetAngle) {
            this.targetAngle = targetAngle;
            this.angleTolerance = Degrees.of(2);
        }
    }
}
