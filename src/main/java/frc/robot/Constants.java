package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class InputConstants {
        public static final int kDriverControllerID = 0;
    }
    
    public static class SwerveModuleConstants {
        public static final double kWheelDiameter = Units.inchesToMeters(4.0);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;
        public static final double kDriveRatio = 6.12 / 1.0;
//        public static final double kDriveRatio = 6.75 / 1.0;
        public static final double kDistancePerMotorRotation = kWheelCircumference / kDriveRatio;

        public static final double kDriveP = .02;
        public static final double kDriveI = 0;
        public static final double kDriveD = 0;

        public static final double kAngleP = .01;
        public static final double kAngleI = 0;
        public static final double kAngleD = 0;

        public static class FL {
            public static final int kDriveID = 27;
            public static final boolean kDriveInverted = false;
            public static final int kAngleID = 28;
            public static final boolean kAngleInverted = false;
            public static final int kEncoderID = 0;
            public static final boolean kEncoderInverted = true;
            public static final double kEncoderOffset = -168.48;
        }

        public static class FR {
            public static final int kDriveID = 21;
            public static final boolean kDriveInverted = false;
            public static final int kAngleID = 24;
            public static final boolean kAngleInverted = false;
            public static final int kEncoderID = 1;
            public static final boolean kEncoderInverted = true;
            public static final double kEncoderOffset = 76.68;
        }

        public static class BL {
            public static final int kDriveID = 25;
            public static final boolean kDriveInverted = false;
            public static final int kAngleID = 26;
            public static final boolean kAngleInverted = false;
            public static final int kEncoderID = 3;
            public static final boolean kEncoderInverted = true;
            public static final double kEncoderOffset = -92.16;
        }

        public static class BR {
            public static final int kDriveID = 29;
            public static final boolean kDriveInverted = false;
            public static final int kAngleID = 22;
            public static final boolean kAngleInverted = false;
            public static final int kEncoderID = 2;
            public static final boolean kEncoderInverted = true;
            public static final double kEncoderOffset = -144;
        }

        public static class SwerveGyroConstants {
            public static final boolean kGyroInverted = true;
            public static final double kGyroOffset = 0;
        }
    }

    public static class SwerveConstants {
        public static final double kWheelOffset = Units.inchesToMeters(10.375);
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[] {
            new Translation2d(kWheelOffset, kWheelOffset),
            new Translation2d(-kWheelOffset, kWheelOffset),
            new Translation2d(kWheelOffset, -kWheelOffset),
            new Translation2d(-kWheelOffset, -kWheelOffset)
        });
        public static final double kModuleRadius = Units.inchesToMeters(14.6725);

        public static final double kDeadband = .1;
        public static final double kMaxVelocity = Units.feetToMeters(16.6);
        public static final double kMaxAngularVelocity = 4;

        public static final boolean fieldCentric = true;

        public static final PIDConstants kTranslationConstants = new PIDConstants(.01);
        public static final PIDConstants kRotationConstants = new PIDConstants(.01);
    }
}
