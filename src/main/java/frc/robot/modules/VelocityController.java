package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;

public class VelocityController {

    private CANSparkMax spark;
    private SparkPIDController pid_controller;
    private RelativeEncoder motor_encoder;

    private double setpoint;
    private double max_output;
    private double max_velocity;

    public VelocityController(CANSparkMax spark_controller, RelativeEncoder encoder, double p, double i, double d,
            double max_output, double max_velocity) {
        /* Initialize spark and encoder objects */
        this.spark = spark_controller;
        this.motor_encoder = encoder;

        /* Get the PID controller from the spark */
        this.pid_controller = spark.getPIDController();
        /* Set the gains */
        this.pid_controller.setP(p);
        this.pid_controller.setI(i);
        this.pid_controller.setD(d);

        this.setpoint = 0;

        this.max_output = max_output;
        this.max_velocity = Math.abs(max_velocity);
    }

    public void set_velocity(double velocity) {
        /* Update the setpoint while limiting to max velocity */
        this.setpoint = MathUtil.clamp(velocity, -this.max_velocity, this.max_velocity);
    }

    public void disable() {
        this.spark.stopMotor();
    }

    public void run() {
        this.pid_controller.setReference(this.setpoint, ControlType.kVelocity);
    }

}
