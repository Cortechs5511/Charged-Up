package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final boolean DIAGNOSTICS = false;
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

        public static final double MAX_VELOCITY = Units.inchesToMeters(13);
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
        
    public static class LimelightConstants {
        public static final double ORIGIN_TO_LIMELIGHT_X = Units.inchesToMeters(2.0);
        public static final double ORIGIN_TO_LIMELIGHT_Y = Units.inchesToMeters(-4.5);
        public static final double ORIGIN_TO_TAG_FINAL = -Units.inchesToMeters(22);
    }

    public static class ArmConstants {
        public static final int LEFT_ID = 30;
        // public static final int RIGHT_ID = 31;
        public static final int EXTENDER_ID = 50;
        public static final int STRING_POT_ID = 0;

        public static final int STRING_POT_RANGE = 100;

        public static final double VOLTAGE_COMPENSATION = 10;
        public static final int CURRENT_LIMIT = 40;
        public static final int CLAW_CURRENT_LIMIT = 40;
        public static final int WINCH_CURRENT_LIMIT = 7;


        public static final double RAMP_RATE = 0.1;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;

        public static final double ROTATE_Kp = 10;
        public static final double ROTATE_Kd = 1;

        public static final double ROTATE_Kg = 2.66;
        public static final double ROTATE_Ks = 0;
        public static final double ROTATE_Kv = 0;
        public static final double ROTATE_Ka = 0;



        public static final double EXTEND_Kp = 0;
        public static final double EXTEND_Kd = 0;

        public static final double AVERAGE_PULLEY_DIAMETER = 0.75;
        
        public static final double ARM_SCORE_TOLERANCE = 0.01;
        public static final double EXTENDER_SCORE_TOLERANCE = 0.1;

        public static final double HIGH_CONE_ROTATIONS = 0.315;
        public static final double MID_CONE_ROTATIONS = 0.26;
        public static final double LOW_CONE_ROTATIONS = 0.109;

        public static final double CONE_SUBSTATION_ROTATIONS = 0.27;
        public static final double CUBE_SUBSTATION_ROTATIONS = 0.248;
        public static final double EXTENDABLE_ROTATIONS = 0.2;
        public static final double INITIAL_ROTATE = 0.05;
        public static final double STOW_ROTATIONS = 0.06;

        
        // -0.1 power fully extended without cone
        public static final double HIGH_POWER = 0.1;
        public static final double MID_POWER = 0.1;
        public static final double LOW_POWER = 0.07;
        public static final double SUBSTATION_POWER = 0.065;

        // Extension preset constants
        // Max 27
        public static final double HIGH_EXTENSION = 6.23;
        public static final double MID_EXTENSION = 46.8;
        public static final double LOW_EXTENSION = 51.4;
        public static final double SUBSTATION_EXTENSION = 78;

        public static final double ZERO_EXTENSION = 76;



        public static final double AUTON_EXTEND_POS = -0.2;
    }

    public static class ClawConstants {
        public static final int CLAW_ID = 31;
    }
}
