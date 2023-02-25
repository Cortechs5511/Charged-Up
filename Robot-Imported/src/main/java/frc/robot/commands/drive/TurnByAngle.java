package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TurnByAngle extends CommandBase {
    private final Drive drive;
    private PIDController turnController = new PIDController(0.01, 0, 0);
    private final double angle;
    double feedForward = 0.1;

    public TurnByAngle(Drive drive, final double angle) {
        this.drive = drive;
        this.angle = angle;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.gyroReset();
        drive.reset(new Pose2d());
        turnController.setTolerance(1.0);
        turnController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        turnController.setSetpoint(angle);
        SmartDashboard.putNumber("Limelight/RobotRotation", drive.getYaw());
        SmartDashboard.putNumber("Limelight/Turn Setpoint", turnController.getSetpoint());

            double output = turnController.calculate(drive.getPose().getRotation().getDegrees());
            // if to the left of target - some margin (5 deg?)
            // output += feed forward
            // if to the right of target + same margin as above
            // output -= feed forward

            if (drive.getYaw() < (turnController.getSetpoint()-5)) {
                output += feedForward;
            } else if (drive.getYaw() > (turnController.getSetpoint() +5)) {
                output -= feedForward;
            }

            if (turnController.atSetpoint()) {
                drive.setPower(0, 0);
            } else {
                output = Math.max(-0.3, Math.min(0.3, output));
                drive.setPower(-output, output);
            }
            SmartDashboard.putNumber("Limelight/Output",output);

    }
    @Override
    public void end(boolean interrupted) {
        drive.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
            if (turnController.atSetpoint()) {
                return true;
            } else{
                return false;
            }
        }
}
