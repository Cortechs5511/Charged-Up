package frc.robot;

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

        public static final double ALIGN_INIT_ANGLE = 18;
        public static final double ALIGN_PID_P = 0.006;
        public static final double ALIGN_PID_I = 0.0;
        public static final double ALIGN_PID_D = 0.001;

        public static final double TURNCONTROLLER_PID_P = 0.01;
        public static final double TURNCONTROLLER_PID_I = 0.0;
        public static final double TURNCONTROLLER_PID_D = 0.0;
        public static final double TURNCONTROLLER_FEEDFORWARD = 0.1;


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

        public static final int FLIP_BUTTON = 2;
        public static final int HALF_SPEED_BUTTON = 2;
        public static final int LIMELIGHT_TOGGLE_BUTTON = 8;
    }

    public static class PneumaticsConstants {
        public static final int PNEUMATICS_MODULE_ID = 17;
    }
        
    public static class LimelightConstants {
        public static final double ORIGIN_TO_LIMELIGHT_X = Units.inchesToMeters(2.0);
        public static final double ORIGIN_TO_LIMELIGHT_Y = Units.inchesToMeters(-4.5);
        public static final double ORIGIN_TO_TAG_FINAL = Units.inchesToMeters(22);
    }

    public static class ArmConstants {
        public static final int LEFT_ID = 30;
        public static final int RIGHT_ID = 31;
        public static final int EXTENDER_ID = 50;

        public static final double VOLTAGE_COMPENSATION = 3;
        public static final int CURRENT_LIMIT = 40;
        public static final double RAMP_RATE = 0.1;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;

        public static final double ROTATE_Kp = 0;
        public static final double ROTATE_Kd = 0;

        public static final double ROTATE_Kg = 2.66;
        public static final double ROTATE_Ks = 0;
        public static final double ROTATE_Kv = 0;
        public static final double ROTATE_Ka = 0;



        public static final double EXTEND_Kp = 0;
        public static final double EXTEND_Kd = 0;

        public static final double MID_CONE_ROTATIONS = 0.25;
        public static final double HIGH_CONE_ROTATIONS = 0.28;
        public static final double SUBSTATION_ROTATIONS = 0.24;
        public static final double EXTENDABLE_ROTATIONS = 0.2;
        public static final double INITIAL_ROTATE = 0.05;

        public static final double AUTON_EXTEND_POS = -0.2;
    }

    public static class ClawConstants {
        
    }
}
