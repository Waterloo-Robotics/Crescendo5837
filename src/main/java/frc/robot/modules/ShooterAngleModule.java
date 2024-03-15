package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class ShooterAngleModule {

    public enum ModuleStates {
        UNKNOWN,
        HOMING,
        HOME,
        ANGLE_SETPOINT;
    }

    public enum RequestStates {
        FIND_HOME,
        HOME,
        AMP_ANGLE,
        SPEAKER_FAR,
        ANGLE_SETPOINT;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initialState = ModuleStates.UNKNOWN;

    public CANSparkMax angle_spark;
    public RelativeEncoder angle_encoder;

    public PIDController angle_pid_controller;
    public double angle_power;

    public double desired_position;

    public boolean home_found;


    public ShooterAngleModule(int pivot_can_id) {
        this.angle_spark = new CANSparkMax(pivot_can_id, MotorType.kBrushless);
        this.angle_spark.setIdleMode(IdleMode.kBrake);
        this.angle_spark.setSmartCurrentLimit(1, 10, 2500);
        this.angle_spark.setOpenLoopRampRate(1);

        this.angle_spark.setSoftLimit(SoftLimitDirection.kForward, 0);
        this.angle_spark.setSoftLimit(SoftLimitDirection.kReverse, 70);

        this.angle_encoder = this.angle_spark.getEncoder();

        this.angle_pid_controller = new PIDController(0.06, 0, 0.01);
        this.angle_pid_controller.setTolerance(2);

        this.angle_power = 0;
        this.desired_position = 0;

        /* Initialize the module to not be homed */
        this.home_found = false;

        this.currentState = ModuleStates.UNKNOWN;
        this.requestedState = RequestStates.HOME;
    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (this.requestedState) {
            case FIND_HOME:
                this.currentState = ModuleStates.HOMING;
                break;

            case AMP_ANGLE:
                /* Only do the PID controller if the home has been found  */
                if (this.home_found) {
                    this.desired_position = 55;
                    this.currentState = ModuleStates.ANGLE_SETPOINT;
                }
                break;
            case SPEAKER_FAR:
                /* Only do the PID controller if the home has been found  */
                if (this.home_found) {
                    this.desired_position = 30;
                    this.currentState = ModuleStates.ANGLE_SETPOINT;
                }
                break;

            case HOME:
                /* Only do the PID controller if the home has been found  */
                if (this.home_found) {
                    this.currentState = ModuleStates.HOME;
                }
                break;

            default:
                break;
        }
    }

    public void update() {
        switch (this.currentState) {
            case HOMING:
                /* Just start going down */
                this.angle_spark.set(-0.06);

                /* If we've hit the limit */
                if (this.angle_spark.getOutputCurrent() > 0.2) {
                    /* Stop the motor */
                    this.angle_spark.set(0);

                    /* Reset the encoder */
                    this.angle_encoder.setPosition(-20);
                    this.home_found = true;

                    this.currentState = ModuleStates.HOME;
                }
                break;
            case HOME:
                /* Get desired power */
                this.angle_power = this.angle_pid_controller.calculate(this.angle_encoder.getPosition(), 0);

                /* Apply Deadband */
                if (this.angle_pid_controller.atSetpoint()) {
                    this.angle_power = MathUtil.applyDeadband(this.angle_power, 0.1);
                }

                /* Limit power */
                this.angle_power = MathUtil.clamp(this.angle_power, -0.4, 0.2);

                this.angle_spark.set(this.angle_power);
                break;

            case ANGLE_SETPOINT:
                /* Get desired power */
                this.angle_power = this.angle_pid_controller.calculate(this.angle_encoder.getPosition(), this.desired_position);

                /* Apply Deadband */
                if (this.angle_pid_controller.atSetpoint()) {
                    this.angle_power = MathUtil.applyDeadband(this.angle_power, 0.1);
                }

                /* Limit power */
                this.angle_power = MathUtil.clamp(this.angle_power, -0.4, 0.5);

                this.angle_spark.set(this.angle_power);

                break;
            
            case UNKNOWN:
                /* If we don't know the state of the shooter angle, just stop the motor */
                this.angle_spark.set(0);
                break;

            default:
                /* Something went wrong, stop the motor */
                this.angle_spark.set(0);
                break;
        }
    }

    public RequestStatusEnum get_request_status() {
        return this.requestStatus;
    }

    public ModuleStates get_state() {
        return this.currentState;
    }

    public void reset_encoder() {
        this.angle_encoder.setPosition(0);
        this.angle_spark.setSoftLimit(SoftLimitDirection.kForward, 0);
        this.angle_spark.setSoftLimit(SoftLimitDirection.kReverse, 70);
    }

}
