package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class stowExtender extends CommandBase {
    private final Extender extender;

    public stowExtender( Extender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {


        extender.goToPosition(1, ArmConstants.ZERO_EXTENSION);


    }

    @Override
    public void end(boolean interrupted) {
        extender.setExtendPower(0);
    }

    @Override
    public boolean isFinished() {
        return ArmConstants.ZERO_EXTENSION - ArmConstants.EXTENDER_SCORE_TOLERANCE <= extender.getStringPotPosition()
        && extender.getStringPotPosition() <= ArmConstants.ZERO_EXTENSION + ArmConstants.EXTENDER_SCORE_TOLERANCE;

    }

}