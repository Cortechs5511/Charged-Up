package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnByAngle extends CommandBase {
    private final Drive drive;
    private PIDController turnController = new PIDController(.03, 0, 0);
    double angle;

    public TurnByAngle(Drive drive, double angle) {
        this.drive = drive;
        this.angle = angle;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        turnController.setSetpoint(angle);
        turnController.setTolerance(1.0);
    }

    @Override
    public void execute() {
            double output = turnController.calculate(drive.getPose().getRotation().getDegrees());


            if (turnController.atSetpoint()) {
                drive.setPower(0, 0);
            } else {
                output = Math.max(-0.3, Math.min(0.3, output));
                drive.setPower(output, -output);
            }
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
