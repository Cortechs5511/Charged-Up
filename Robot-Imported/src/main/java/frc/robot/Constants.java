package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final boolean DIAGNOSTICS = true;
    public static final double DWELL_PERIOD = 0.5;

    public static class DriveConstants {
        public static final int LEFT_LEADER_ID = 10;
        public static final int LEFT_FOLLOWER_ID = 51;
        public static final int RIGHT_LEADER_ID = 20;
        public static final int RIGHT_FOLLOWER_ID = 11;

        public static final double VOLTAGE_COMPENSATION = 11;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
        public static final double RAMP_RATE = 0.05;
        public static final int CURRENT_LIMIT = 80;

        public static final double GEARING = (44f / 24f) * (68f / 11f);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double ENCODER_TO_METERS = WHEEL_DIAMETER / GEARING;
        public static final double DRIVE_VELOCITY_TOLERANCE = 1;

        public static final double PID_MIN_OUTPUT = 0;
        public static final double PID_MAX_OUTPUT = 0;
        public static final double PID_P = 0;
        public static final double PID_I = 0;
        public static final double PID_D = 0;
        public static final double PID_FF = 0;

        public static final double ANGLE_P = 0.0;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.0;
        public static final double ANGLE_PID_LIMIT = 0.3;

        public static final double TRACK_WIDTH = 0.6858;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

        public static final double Ks = 0.20163;
        public static final double Kv = 2.9383;
        public static final double Ka = 0.45516;
        public static final double Kp = 0.005668;
    }

    public static class OIConstants {
        public static final int LEFT_STICK_PORT = 0;
        public static final int RIGHT_STICK_PORT = 1;
        public static final int XBOX_CONTROLLER_PORT = 2;

        public static final double DEADBAND = 0.05;

        public static final int WRIST_UP_AXIS = 2;
        public static final int WRIST_DOWN_AXIS = 3;
        public static final int FLIP_BUTTON = 2;
        public static final int HALF_SPEED_BUTTON = 2;
        public static final int LIMELIGHT_TOGGLE_BUTTON = 8;
    }

    public static class LimelightConstants {
        public static final double ORIGIN_TO_LIMELIGHT_X = Units.inchesToMeters(2.0);
        public static final double ORIGIN_TO_LIMELIGHT_Y = Units.inchesToMeters(-4.5);
        public static final double ORIGIN_TO_TAG_FINAL = Units.inchesToMeters(22);
    }

    public static class ArmConstants {
        public static final int LEFT_ID = 30;
        public static final int RIGHT_ID = 31;

        public static final double VOLTAGE_COMPENSATION = 3;
        public static final int CURRENT_LIMIT = 40;
        public static final double RAMP_RATE = 0.05;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> ecc6e08d479b12293b4235537aaabe0c5175f218
