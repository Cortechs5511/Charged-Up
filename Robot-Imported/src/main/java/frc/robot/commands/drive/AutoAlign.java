package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class AutoAlign extends CommandBase {
    private final Drive drive;
    private PIDController levelPID = new PIDController(0.0075,0,0.003);
    public AutoAlign(Drive subsystem) {
        drive = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        levelPID.setSetpoint(0.0);
    }

    @Override
    public void execute() {
            double angle = drive.getPitch();
            double output = levelPID.calculate(angle);


            if (Math.abs(angle) < 8) {
                drive.setPower(0, 0);
            } else {
                output = Math.max(-0.3, Math.min(0.3, output));
                drive.setPower(-output, -output);
            }
    }
    @Override
    public void end(boolean interrupted) {
        drive.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
            return false;
    }
}
