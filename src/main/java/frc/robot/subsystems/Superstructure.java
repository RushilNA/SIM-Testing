package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.*;

public class Superstructure extends SubsystemBase {

    public enum SuperstructureState {
        START,
        INTAKE,
        L1_SCORING,
        L2_SCORING,
        L3_SCORING,
        L4_SCORING
    }

    private final Arm arm;
    private final Elevator elevator;

    private SuperstructureState currentState = SuperstructureState.START;

    public Superstructure(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    public void setState(SuperstructureState newState) {
        if (newState != currentState) {
            currentState = newState;

            // Tell both subsystems what to do for the new state
            switch (newState) {
                case START:
                    arm.setState(ArmMode.STOP);
                    elevator.setState(ElevatorMode.STOP);
                    break;
                case INTAKE:
                    arm.setState(ArmMode.INTAKE);
                    elevator.setState(ElevatorMode.INTAKE);
                    break;
                case L1_SCORING:
                    arm.setState(ArmMode.L1);
                    elevator.setState(ElevatorMode.L1);
                    break;
                case L2_SCORING:
                    arm.setState(ArmMode.L2);
                    elevator.setState(ElevatorMode.L2);
                    break;
                case L3_SCORING:
                    arm.setState(ArmMode.L3);
                    elevator.setState(ElevatorMode.L3);
                    break;
                case L4_SCORING:
                    arm.setState(ArmMode.L4);
                    elevator.setState(ElevatorMode.L4);
                    break;
            }
        }
    }

    public SuperstructureState getState() {
        return currentState;
    }

    /** Returns true if both subsystems are at their setpoints */
    public boolean isAtTarget() {
        return arm.isAtTarget() && elevator.isAtTarget();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Superstructure State", currentState.name());
        SmartDashboard.putBoolean("Superstructure At Target", isAtTarget());
    }
}
