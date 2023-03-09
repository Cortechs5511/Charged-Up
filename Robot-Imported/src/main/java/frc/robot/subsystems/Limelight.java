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
    private Trajectory trajectory = new Trajectory();
    private Pose2d startingPose = new Pose2d(0,0, new Rotation2d());

        
    public Limelight() {

    }
    public void takeSnapshot() {
        table.getEntry("snapshot").setNumber(1);
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
        

        if (hasTargets()) {

                double rotate = getPitch();
                double xLL = -getZ();
                double yLL = -getX();
                
                Rotation2d robotFinalToRobotInitial = new Rotation2d(Units.degreesToRadians(rotate));
    
                Translation2d originFinalToTag = new Translation2d(LimelightConstants.ORIGIN_TO_TAG_FINAL, Units.inchesToMeters(sideOffset));
    
                Translation2d limelightToTag = new Translation2d(xLL, yLL);
    
                Translation2d originToLimelight = new Translation2d(
                LimelightConstants.ORIGIN_TO_LIMELIGHT_X, 
                LimelightConstants.ORIGIN_TO_LIMELIGHT_Y);
    
                Translation2d originToTag = limelightToTag.plus(originToLimelight);
    
                Translation2d finalTranslation = originToTag.minus(originFinalToTag.rotateBy(robotFinalToRobotInitial));
    
                SmartDashboard.putNumber("Limelight/rotate", -getPitch());
                SmartDashboard.putNumber("Limelight/translateX", finalTranslation.getX());
                SmartDashboard.putNumber("Limelight/translateY", finalTranslation.getX());
    
    
    
    
                Pose2d endingPose = new Pose2d(finalTranslation.getX(), finalTranslation.getY(), new Rotation2d());
    
                SmartDashboard.putString("Limelight/Endingpose", endingPose.toString());
                SmartDashboard.putString("Limelight/Startpose", startingPose.toString());
    
                var interiorWaypoints = new ArrayList<Translation2d>();
                interiorWaypoints.add(new Translation2d(endingPose.getX() / 3.0, endingPose.getY() / 3.0));
                interiorWaypoints.add(new Translation2d(2.0 * endingPose.getX() / 3.0, 2.0 * endingPose.getY() / 3.0));
    
                TrajectoryConfig config = new TrajectoryConfig(0.5, 0.25);
                config.setReversed(false);
                
                trajectory = TrajectoryGenerator.generateTrajectory(
                    startingPose,
                    interiorWaypoints,
                    endingPose,
                    config);
        }
    }
}

