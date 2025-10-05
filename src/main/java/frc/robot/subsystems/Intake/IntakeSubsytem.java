// package frc.robot.subsystems.Intake;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /**
//  * IntakeSubsytem (current-sense version)
//  *
//  * <p>Changes from your original: - REMOVED CANrange usage entirely. - Game piece detection logic executes ONLY
// inside
//  * hasGamePiece(). - periodic() just publishes telemetry and the last cached detection state.
//  *
//  * <p>API: - setIntakeSpeed(double speed): + is inward intake - setIntakePivoit(double rotations): pivot setpoint in
//  * TalonFX motor rotations (used by MotionMagic) - hasGamePiece(): reads roller stator current, debounces spike,
// returns
//  * true on confirm - resetDetection(): clears debounce/cached state (optional to call when starting a new intake)
//  */
// public class IntakeSubsytem extends SubsystemBase {
//     private final TalonFX intakepivoit = new TalonFX(30);
//     private final TalonFX roller = new TalonFX(31);

//     // Config
//     private final TalonFXConfiguration cfg = new TalonFXConfiguration();
//     private final Slot0Configs slot0 = cfg.Slot0;
//     private final MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
//     private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

//     // Detection state (used ONLY by hasGamePiece())
//     private final Timer detectTimer = new Timer();
//     private boolean aboveThreshold = false;
//     private boolean detectedCached = false; // last computed value from hasGamePiece()
//     private double rollerCmd = 0.0; // last commanded roller speed (to gate detection)

//     // === TUNE THESE ===
//     public static final class DetectCfg {
//         /** Consider intake "inward" if commanded roller speed is above this. */
//         public static final double MIN_INWARD_CMD = 0.2;
//         /** Amps threshold for “we bit something”. Start ~15–22 A depending on your roller. */
//         public static final double CURRENT_SPIKE_A = 18.0;
//         /** Require the spike to persist this long to confirm (debounce). */
//         public static final double CONFIRM_TIME_S = 0.06;
//     }

//     public IntakeSubsytem() {
//         // MotionMagic gains (kept from your version)
//         slot0.kG = 0.3;
//         slot0.kS = 0.3;
//         slot0.kV = 0.3;
//         slot0.kA = 0.01;
//         slot0.kP = 8;
//         slot0.kI = 0;
//         slot0.kD = 0.3;

//         motionMagicConfigs.MotionMagicCruiseVelocity = 500;
//         motionMagicConfigs.MotionMagicAcceleration = 400;
//         motionMagicConfigs.MotionMagicJerk = 900;

//         intakepivoit.getConfigurator().apply(cfg);
//         intakepivoit.setNeutralMode(NeutralModeValue.Brake);
//         roller.setNeutralMode(NeutralModeValue.Brake);

//         detectTimer.stop();
//         detectTimer.reset();
//     }

//     @Override
//     public void periodic() {
//         // Telemetry only — NO detection logic here
//         SmartDashboard.putNumber(
//                 "Intake Position (rot)", intakepivoit.getPosition().getValueAsDouble());
//         SmartDashboard.putNumber("Roller Velocity (rps)", roller.getVelocity().getValueAsDouble());
//         SmartDashboard.putNumber(
//                 "Roller Stator Current (A)", roller.getStatorCurrent().getValueAsDouble());
//         SmartDashboard.putBoolean("Intake Piece Detected (cached)", detectedCached);
//     }

//     /** Roller speed command: positive = inward intake. */
//     public void setIntakeSpeed(double speed) {
//         rollerCmd = speed;
//         roller.set(speed);
//     }

//     /** Pivot setpoint (in TalonFX sensor rotations). */
//     public void setIntakePivoit(double positionRotations) {
//         intakepivoit.setControl(m_request.withPosition(positionRotations));
//     }

//     /** Convenience Command wrapper. */
//     public Command setIntakePosition(double positionRotations) {
//         return run(() -> setIntakePivoit(positionRotations));
//     }

//     public void stopIntake() {
//         rollerCmd = 0.0;
//         intakepivoit.set(0);
//         roller.set(0);
//     }

//     /** Clear debounce/cached detection state (call when starting a new intake cycle if desired). */
//     public void resetDetection() {
//         aboveThreshold = false;
//         detectedCached = false;
//         detectTimer.stop();
//         detectTimer.reset();
//     }

//     /**
//      * Detection runs ONLY here. Call frequently while intaking. Uses roller stator current spike + debounce to
// confirm
//      * capture.
//      */
//     public boolean hasGamePiece() {
//         boolean inward = rollerCmd > DetectCfg.MIN_INWARD_CMD;

//         double amps = roller.getStatorCurrent().getValueAsDouble();

//         if (inward && amps >= DetectCfg.CURRENT_SPIKE_A) {
//             if (!aboveThreshold) {
//                 aboveThreshold = true;
//                 detectTimer.restart();
//             }
//         } else {
//             aboveThreshold = false;
//             detectTimer.stop();
//             detectTimer.reset();
//         }

//         detectedCached = aboveThreshold && detectTimer.hasElapsed(DetectCfg.CONFIRM_TIME_S);
//         return detectedCached;
//     }
// }
