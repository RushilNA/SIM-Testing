// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UpperBoddy;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls a dual-motor elevator mechanism for game piece manipulation. It supports multiple
 * distances for different game actions and provides both open-loop and closed-loop control options.
 */
public class Elevator extends SubsystemBase {
    // Hardware interface and inputs
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    public enum wantedStates {
        home,
        idle,
        move_to_position
    }

    public enum curentStates {
        homing,
        idling,
        moving_to_target
    }

    private wantedStates wState = wantedStates.idle;
    private curentStates cState = curentStates.idling;

    // Current elevator distance mode

    // Alerts for motor connection status
    private final Alert leaderMotorAlert = new Alert("Elevator leader motor isn't connected", AlertType.kError);

    /**
     * Creates a new Elevator subsystem with the specified hardware interface.
     *
     * @param io The hardware interface implementation for the elevator
     */
    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // Update and log inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Subsystem1/Elevator/Wstate", wState);
        Logger.recordOutput("Subsystem1/Elevator/Cstate", cState);

        Logger.recordOutput("States/ElevatorWheelsSystemState", cState);
        Logger.recordOutput("States/ElevatorWheelsWantedState", wState);

        // Update motor connection status alerts
        leaderMotorAlert.set(!inputs.leaderConnected);
    }

    /**
     * Returns the current distance of the elevator.
     *
     * @return The current angular distance
     */
    @AutoLogOutput
    public Distance getPosition() {
        return inputs.elevatorDistance;
    }

    private void handleStateTransition() {
        switch (wState) {
            case idle:
                cState = curentStates.idling;

                break;
            case home:
                cState = curentStates.homing;
                break;
            case move_to_position:
                cState = curentStates.moving_to_target;
                break;

            default:
                break;
        }
    }

    /**
     * Factory methods for elevator positions
     *
     * <p>Scoring positions: - L1: Low scoring - L2: Mid scoring - L3: High scoring - L4: Extended high scoring
     *
     * <p>Other positions: - intake: Intake position - stop: Emergency stop
     */

    // === Helpers for Uperbody ===
    public void setDistance(Distance distance) {
        this.wState = wantedStates.move_to_position;

        io.setDistance(distance);
    }

    public boolean atDistance(Distance distance) {
        return getPosition().isNear(distance, UpperBoddy.kElevatorTol);
    }
}
