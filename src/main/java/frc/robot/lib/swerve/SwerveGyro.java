package frc.robot.lib.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveGyro {
    private AHRS gyro;
    private double offset;
    private boolean inverted;

    public SwerveGyro(double offset, boolean inverted) {
        this.gyro = new AHRS();
        this.offset = offset;
        this.inverted = inverted;
    }

    public Rotation2d getRotation2d() {
        if(inverted) {
            return Rotation2d.fromDegrees(gyro.getYaw() - offset).unaryMinus();
        }
        return Rotation2d.fromDegrees(gyro.getYaw() - offset);
    }
}
