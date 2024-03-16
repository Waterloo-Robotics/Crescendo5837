package frc.robot.modules;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class FlywheelSubmodule {

    public enum ModuleStates {
        STOPPED,
        SPIN_UP,
        AT_SPEED,
        SPIN_DOWN,
        SATURATED_SPEED_NOT_MET;
    }

    public enum RequestStates {
        STOP,
        SPIN_UP_SPEAKER,
        SPIN_UP_AMP;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE,
        INCOMPLETE;
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initalState = ModuleStates.STOPPED;

    public CANSparkMax right_flywheel_spark;
    public RelativeEncoder right_encoder;

    public CANSparkMax left_flywheel_spark;
    public RelativeEncoder left_encoder;

    private static final double right_setpoint_speaker = 1;
    private static final double left_setpoint_speaker = -1;

    private static final double right_setpoint_amp = 0.10;
    private static final double left_setpoint_amp = -0.10;

    public double right_setpoint;
    public double left_setpoint;

    public FlywheelSubmodule (int right_id, int left_id) {
        this.currentState = ModuleStates.STOPPED;
        this.requestedState = RequestStates.STOP;

        this.right_flywheel_spark = new CANSparkMax(right_id, CANSparkLowLevel.MotorType.kBrushless);
        this.right_flywheel_spark.setOpenLoopRampRate(1);
        this.right_encoder = this.right_flywheel_spark.getEncoder();

        this.left_flywheel_spark = new CANSparkMax(left_id, CANSparkLowLevel.MotorType.kBrushless);
        this.left_flywheel_spark.setOpenLoopRampRate(1);
        this.left_encoder = this.left_flywheel_spark.getEncoder();
    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (this.requestedState) {
            case STOP:
                this.currentState = ModuleStates.STOPPED;
                break;

            case SPIN_UP_SPEAKER:
                this.currentState = ModuleStates.SPIN_UP;
                this.right_setpoint = right_setpoint_speaker;
                this.left_setpoint = left_setpoint_speaker;
                break;
            case SPIN_UP_AMP:
                this.currentState = ModuleStates.SPIN_UP;
                this.right_setpoint = right_setpoint_amp;
                this.left_setpoint = left_setpoint_amp;
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
            case SPIN_UP:
                this.currentState = ModuleStates.AT_SPEED;
                /* Purposefully didn't add a break here so it also sets the speed */
            case AT_SPEED:
                this.right_flywheel_spark.set(right_setpoint);
                this.left_flywheel_spark.set(left_setpoint);
                break;
            case SPIN_DOWN:
                this.currentState = ModuleStates.STOPPED;
                /* Purposefully didn't add a break here so it also sets the speed */
            case STOPPED:
                this.right_flywheel_spark.set(0);
                this.left_flywheel_spark.set(0);
        }
    }

}
