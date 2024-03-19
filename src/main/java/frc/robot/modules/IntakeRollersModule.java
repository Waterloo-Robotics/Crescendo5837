package frc.robot.modules;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollersModule {

    public enum ModuleStates {
        STOPPED,
        WAITING_FOR_NOTE,
        POSITIONING_NOTE,
        NOTE_IN_POSITION,
        TRANSFER_NOTE,
        EMPTYING_INTAKE;
    }

    public enum RequestStates {
        STOP,
        POSITION_NOTE,
        INTAKE_NOTE,
        EMPTY_INTAKE,
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

    double kP = 0.03;
    double kI = 0.0;
    double kD = 0.0;

    public final static ModuleStates initialState = ModuleStates.STOPPED;

    public CANSparkMax intakeRollerMotor;
    public RelativeEncoder intakeEncoder;

    PIDController intakeRollerPID;
    double intakePIDPower = 0;

    Timer roller_timer;

    public IntakeRollersModule(int intakeMotorID) {
        this.intakeRollerMotor = new CANSparkMax(intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
        this.intakeRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        /* Get Encoder */
        this.intakeEncoder = this.intakeRollerMotor.getEncoder();
        this.intakeEncoder.setPositionConversionFactor(10);

        this.intakeRollerPID = new PIDController(kP, kI, kD);
        this.intakeRollerPID.setTolerance(5);
        this.roller_timer = new Timer();

        this.currentState = ModuleStates.STOPPED;
        this.requestedState = RequestStates.STOP;
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

            case POSITION_NOTE:
                this.currentState = ModuleStates.POSITIONING_NOTE;
                this.intakeRollerMotor.getEncoder().setPosition(0);
                this.intakeRollerPID.setSetpoint(230);
                break;

            case EMPTY_INTAKE:
                this.currentState = ModuleStates.EMPTYING_INTAKE;
                this.roller_timer.restart();
                break;
            
            case TRANSFER_NOTE:
                this.currentState = ModuleStates.TRANSFER_NOTE;
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

        switch (this.currentState) {

            case WAITING_FOR_NOTE:
                intakeRollerMotor.set(1);
                break;

            case STOPPED:
                intakeRollerMotor.set(0);
                break;

            case POSITIONING_NOTE:
                this.intakePIDPower = MathUtil.clamp(intakeRollerPID.calculate(this.intakeEncoder.getPosition()), -0.1, 0.1);
                intakeRollerMotor.set(this.intakePIDPower);

                /* If the roller has reached position, stop the intake */
                if (this.intakeRollerPID.atSetpoint())
                {   
                    this.request_state(RequestStates.STOP);
                }

                SmartDashboard.putNumber("Error", intakeRollerPID.getPositionError());
                SmartDashboard.putNumber("Commanded Power", intakePIDPower);
                SmartDashboard.putNumber("Setpoint", intakeRollerPID.getSetpoint());
                SmartDashboard.putNumber("Current Position", intakeRollerMotor.getEncoder().getPosition());
                break;

            case EMPTYING_INTAKE:
                SmartDashboard.putString("Empty Intake", "3");
                if (this.roller_timer.hasElapsed(0.5)) {
                    /* Intake has been empied, stop the intake and transistion into the empty state */
                    this.roller_timer.stop();
                    this.currentState = ModuleStates.STOPPED;
                } else {
                    /* Reverse the intake */
                    intakeRollerMotor.set(-1);
                }
                break;

            case TRANSFER_NOTE:
                intakeRollerMotor.set(1);
                break;

        }

        lastState = currentState;

    }

}
