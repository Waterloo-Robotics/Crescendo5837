package frc.robot.modules;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollersModule {

    public enum ModuleStates {
        STOPPED,
        WAITING_FOR_NOTE,
        POSITIONING_NOTE,
        NOTE_IN_POSITION,
        TRANSFER_NOTE;
    }

    public enum RequestStates {
        STOP,
        POSITION_NOTE,
        INTAKE_NOTE,
        RESET,
        TRANSFER_NOTE;
    }

    public enum RequestStatusEnum {
        COMPLETE,
        IN_PROGRESS,
        FAILED;
    }

    public ModuleStates currentState;
    public RequestStates requestedState = RequestStates.STOP;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    double kP = 0.05;
    double kI = 0.0;
    double kD = 0.0;

    public final static ModuleStates initialState = ModuleStates.STOPPED;

    public CANSparkMax intakeRollerMotor;
    PIDController intakeRollerPID;
    double intakePIDPower = 0;

    public IntakeRollersModule(int intakeMotorID) {
        this.intakeRollerMotor = new CANSparkMax(intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
        this.intakeRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.intakeRollerPID = new PIDController(kP, kI, kD);
    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case INTAKE_NOTE:
                this.currentState = ModuleStates.WAITING_FOR_NOTE;
                break;

            case STOP:
                this.currentState = ModuleStates.STOPPED;
                break;

            case RESET:
                this.intakeRollerMotor.getEncoder().setPosition(0);
                break;

            case POSITION_NOTE:
                this.currentState = ModuleStates.POSITIONING_NOTE;
                intakeRollerPID.setSetpoint(1);
                break;

        }

    }

    public RequestStatusEnum get_request_status() {
        return this.requestStatus;
    }

    public ModuleStates get_state() {
        return this.currentState;
    }

    public void update() {

        SmartDashboard.putString("Intake Rollers Current State", String.valueOf(this.currentState));

        switch (this.currentState) {

            case WAITING_FOR_NOTE:
                intakeRollerMotor.set(1);
                break;

            case STOPPED:
                intakeRollerMotor.set(0);
                break;

            case POSITIONING_NOTE:
//                intakeRollerMotor.set(0);
                intakePIDPower = intakeRollerPID.calculate(this.intakeRollerMotor.getEncoder().getPosition());
                intakeRollerMotor.set(intakePIDPower);
                SmartDashboard.putNumber("Error", intakeRollerPID.getPositionError());
                SmartDashboard.putNumber("Commanded Power", intakePIDPower);
                SmartDashboard.putNumber("Setpoint", intakeRollerPID.getSetpoint());
                SmartDashboard.putNumber("Current Position", intakeRollerMotor.getEncoder().getPosition());
                break;

        }

        lastState = currentState;

    }

}
