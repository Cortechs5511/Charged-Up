package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.OIConstants;

public class OI {
    private static OI oi;

    public final Joystick leftStick = new Joystick(OIConstants.LEFT_STICK_PORT);
    public final Joystick rightStick = new Joystick(OIConstants.RIGHT_STICK_PORT);
    public final XboxController controller = new XboxController(OIConstants.XBOX_CONTROLLER_PORT);

    private OI() {
    }

    /**
     * Singleton for getting instance of operator input
     *
     * @return OI object of self
     */
    public static OI getInstance() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    /**
     * Returns number [-1.0, 1.0] of left joystick input
     *
     * @return double left stick Y axis
     */
    public double getLeftY() {
        return leftStick.getY();
    }

    /**
     * Returns number [-1.0, 1.0] of right joystick input
     *
     * @return double right stick Y axis
     */
    public double getRightX() {
        return rightStick.getX();
    }

    public double getLeftX() {
        return leftStick.getX();
    }

    public double getRightY() {
        return rightStick.getY();
    }

    /**
     * Returns power for arm 
     * 1 or 1 if stick is past deadband in both directions
     * 0 if stick is within deadband
     * 
     * @return double controller left joystick power
     */
    public double getArmPower() {
        double power = controller.getRawAxis(1);
        return power;
    }

    /**
     * Returns the value of left joystick with values within deadband truncated
     *
     * @return double value of joystick
     */
    public double getLeftYDeadband() {
        double leftY = getLeftY();
        if (Math.abs(leftY) < OIConstants.DEADBAND) {
            return 0;
        }

        return leftY;
    }

    /**
     * Returns the value of left joystick with values within deadband truncated
     *
     * @return double value of joystick
     */
    public double getRightXDeadband() {
        double rightX = getRightX();
        if (Math.abs(rightX) < OIConstants.DEADBAND) {
            return 0;
        }

        return rightX;
    }

    public double getLeftXDeadband() {
        double leftX = getLeftX();
        if (Math.abs(leftX) < OIConstants.DEADBAND) {
            return 0;
        }

        return leftX;
    }


    public double getRightYDeadband() {
        double rightY = getRightY();
        if (Math.abs(rightY) < OIConstants.DEADBAND) {
            return 0;
        }

        return rightY;
    }


    /**
     * Sets rumble value of controller to specified intensity
     * 
     * @param intensity double value to set up to 1
     */
    public void setRumble(double intensity) {
        controller.setRumble(RumbleType.kLeftRumble, intensity);
        controller.setRumble(RumbleType.kRightRumble, intensity);
    }

}