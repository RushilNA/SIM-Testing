// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class armtest extends SubsystemBase {
    private final TalonFX clawMotor = new TalonFX(20);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(5, 0, 1, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            // Feedforward Constants
            .withFeedforward(new ArmFeedforward(0.3, 0, 1.2))
            .withSimFeedforward(new ArmFeedforward(0.3, 1.75, 1.95))
            // Telemetry name and verbosity level
            .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox
            // attached to your motor.
            .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    // Vendor motor controller object
    private TalonFX talon = new TalonFX(30);

    // Create our SmartMotorController from our Spark and config with the NEO.
    private SmartMotorController sparkSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

    private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(Degrees.of(-20), Degrees.of(10))
            // Hard limit is applied to the simulation.
            .withHardLimit(Degrees.of(-30), Degrees.of(40))
            // Starting position is where your arm starts
            .withStartingPosition(Degrees.of(-5))
            // Length and mass of your arm for sim.
            .withLength(Feet.of(3))
            .withMass(Pounds.of(1))
            // Telemetry name and verbosity for the arm.
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Arm arm = new Arm(armCfg);

    /**
     * Set the angle of the arm.
     *
     * @param angle Angle to go to.
     */
    public Command setAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    /**
     * Move the arm up and down.
     *
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) {
        return arm.set(dutycycle);
    }

    /** Run sysId on the {@link Arm} */
    public Command sysId() {
        return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    /** Creates a new ExampleSubsystem. */
    public armtest() {}

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            /* one-time action goes here */
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        arm.updateTelemetry();
        SmartDashboard.putNumber("Arm Position", arm.getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        arm.simIterate();
    }

    // private void setClawSpeed(double speed) {
    //     clawMotor.set(speed);
    // }

    public Angle getAngle() {
        return arm.getAngle();
    }

    //     public Command setClawRoller(double speed) {
    //         return run(() -> setClawSpeed(speed)); // Example speed, adjust as needed
    //     }
}
