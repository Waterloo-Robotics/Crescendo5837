package frc.robot.lib.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveEncoder {
    private CANcoder encoder;
    private double offset;
    private boolean inverted;

    /** Thriftybot encoder
     * @param id 
     * Analog channel
     * @param offset
     * double, -180 to 180
     */
    public SwerveEncoder(int id, double offset, boolean inverted) {
        encoder = new CANcoder(id);
        this.offset = offset;
    }

    /** Converts the encoder output to -180 to 180 */
    private double convertEncoderToDegrees() {
        double absPosition = encoder.getAbsolutePosition().getValue();
        if(inverted) {
            absPosition = 1 - absPosition;
        }
        absPosition *= 360;
        absPosition -= 180;
        return absPosition - offset;
    }

    /** Return a rotation2d of the encoder reading, with offset considered, from -180 to 180 deg */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(convertEncoderToDegrees());
    }
}