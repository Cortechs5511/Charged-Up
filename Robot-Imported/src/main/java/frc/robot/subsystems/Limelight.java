package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Limelight extends SubsystemBase {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final double[] botPose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    private final double[] tagId = table.getEntry("tid").getDoubleArray(new double[6]);
    private double sideOffset = 0;
    private static boolean flag = true;
    private static Trajectory trajectory = new Trajectory();

        
    public Limelight() {
        // setting in picture in picture mode main
        // table.getEntry("stream").setNumber(1);
    }
    public void takeSnapshot() {
        table.getEntry("snapshot").setNumber(1);
    }

    public void resetSnapshot() {
        table.getEntry("snapshot").setNumber(0);
    }


    public double getX() {
        //return botPose[0] doesn't even update periodically
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    }

    public double getY() {
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
    }

    public double getZ() {
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];
    }

    public double getRoll() {
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[3];
    }

    public double getPitch() {
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    }

    public double getYaw() {
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[5];
    }
    // public double getLatency() {
    //     return table.getEntry("botpose").getDoubleArray(new double[6])[6];
    // }

    public boolean hasTargets () {
        if (table.getEntry("tv").getDouble(0) == 1) {
            return true;
        } else {
            return false;
        }
    }

    public void setSideOffset(double newSideOffset) {
        sideOffset = newSideOffset;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public void setTrajectory(Trajectory traj) {
        trajectory = traj;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/HasTargets", hasTargets());
            SmartDashboard.putNumber("Limelight/X", getX());
            SmartDashboard.putNumber("Limelight/Y", getY());
            SmartDashboard.putNumber("Limelight/Z", getZ());
            SmartDashboard.putNumber("Limelight/Roll", getRoll());
            SmartDashboard.putNumber("Limelight/Pitch", getPitch());
            SmartDashboard.putNumber("Limelight/yaw", getYaw());
            //SmartDashboard.putNumber("Limelight/Latency", getLatency());
        
    }
}

