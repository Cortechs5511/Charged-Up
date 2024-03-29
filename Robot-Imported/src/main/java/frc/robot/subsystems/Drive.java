package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.Zero;

public class Drive extends SubsystemBase {
    private static final double DRIVE_TO_M = 0.0497;
    private static final double DRIVE_TO_V = .01;
    private final CANSparkMax rightLeader = createDriveController(DriveConstants.LEFT_LEADER_ID, true);
    private final CANSparkMax rightFollower = createDriveController(DriveConstants.LEFT_FOLLOWER_ID, true);

    private final CANSparkMax leftLeader = createDriveController(DriveConstants.RIGHT_LEADER_ID, false);
    private final CANSparkMax leftFollower = createDriveController(DriveConstants.RIGHT_FOLLOWER_ID, false);

    private final RelativeEncoder leftEncoder = createEncoder(leftFollower);
    private final RelativeEncoder rightEncoder = createEncoder(rightFollower);
    private double maxPower = 1.0;

    private final AHRS gyro = new AHRS(Port.kMXP);
    private final DifferentialDriveOdometry odometry;
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv, DriveConstants.Ka);

    private boolean inverted = false;


    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

    private final Field2d field = new Field2d();


    public  Drive() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftEncoder.setPositionConversionFactor(DRIVE_TO_M);
        rightEncoder.setPositionConversionFactor(DRIVE_TO_M);
        leftEncoder.setVelocityConversionFactor(DRIVE_TO_V);
        rightEncoder.setVelocityConversionFactor(DRIVE_TO_V);
        zero();
        SmartDashboard.putData("zero" , new Zero(this)) ;
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), maxPower, maxPower);
        odometry.resetPosition(gyro.getRotation2d(),leftEncoder.getPosition(),rightEncoder.getPosition(), new Pose2d());
        SmartDashboard.putData("Drivetrain/Field", field);
    }

    public void arcadeDrive(double moveSpeed, double rotateSpeed) {
        differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);

    } 

    public void speedDrive(double leftSpeed, double rightSpeed) {
        double leftVolts = driveFeedforward.calculate(leftSpeed);
        double rightVolts = driveFeedforward.calculate(rightSpeed);

        setVolts(leftVolts, rightVolts);
        differentialDrive.feed();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed, false);
    }


    /**
     * Get position of left encoder
     * 
     * @return double encoder sensed position, meters
     */
    public void zero() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        gyro.reset();
        gyro.calibrate();
        
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void gyroReset() {
        gyro.reset();
    }
    
    public double getYaw() {
        return gyro.getYaw();

    }

    public double getRoll() {
        return gyro.getRoll();
    }
    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    /**
     * Getter for field2d
     * @return Field2d field
     */
    public Field2d getField2d() {
        return field;
    }

    /**
     * Get position of right encoder
     * 
     * @return double encoder sensed position, meters
     */
    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    /**
     * Get velocity of left encoder
     * Note that this has a ~100ms phase lag
     * 
     * @return double encoder velocity, meters per second
     */
    public double getLeftVelocity() {
        return leftEncoder.getVelocity() ;
    }

    /**
     * Get velocity of right encoder
     * Note that this has a ~100ms phase lag
     * 
     * @return double encoder velocity, meters per second
     */
    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    /**
     * Returns wheel speeds for autonomous
     *
     * @return DifferentialDriveWheelSpeeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    /**
     * Resets odometry for auto
     */
     public void resetOdometry() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d());
    } 

    public void reset(Pose2d pose) {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
            odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
        }

        
    /**
     * Returns Pose2D of robot odometry
     * 
     * @return Pose2D robot position (x, y) in meters
     */
     public Pose2d getPose() {
        return odometry.getPoseMeters();
    } 

    /**
     * Sets drivetrain inversion flag to opposite of existing value
     */
    public void flip() {
        inverted = !inverted;
    }

    /**
     * Sets drivetrain set() limitation interpolation value for both motors
     *
     * @param power double max power value out of 1
     * @return 
     */
    public void setMaxPower(double power) {
        maxPower = power;
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Commands setpoint value out of 1 (voltage compensated) on both motors
     *
     * @param leftSpeed  double left motor setpoint out of maxPower
     * @param rightSpeed double right motor setpoint out of maxPower
     */
    public void setPower(double leftSpeed, double rightSpeed) {
        if (!inverted) {
            leftLeader.set(leftSpeed * maxPower);
            rightLeader.set(rightSpeed * maxPower);
            differentialDrive.feed();
        } else {
            rightLeader.set(-leftSpeed * maxPower);
            leftLeader.set(-rightSpeed * maxPower);
            differentialDrive.feed();
        }
    }

    public double getLeftPower() {
        return leftLeader.get();
    }

    public double getRightPower() {
        return leftLeader.get();
    }
    /**
     * Sets desired output voltage for drivetrain
     * 
     * @param leftVolts  double volts for left side
     * @param rightVolts double volts for right side
     */
    public void setVolts(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    /**
     * Creates a CANSparkMax controller with preferred settings
     *
     * @param port       applicable CAN port
     * @param isInverted boolean inversion flag
     * @return CANSparkMax controller
     */
    private CANSparkMax createDriveController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(DriveConstants.VOLTAGE_COMPENSATION);
        controller.setIdleMode(DriveConstants.IDLE_MODE);
        controller.setOpenLoopRampRate(DriveConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(DriveConstants.RAMP_RATE);

        controller.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
        controller.setSecondaryCurrentLimit(100);

        controller.setInverted(isInverted);
        return controller;
    }

    /**
     * Creates an encoder object from NEO with preferred settings
     *
     * @param controller CANSparkMax controller
     * @return RelativeEncoder
     */
    private RelativeEncoder createEncoder(CANSparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();

        encoder.setPositionConversionFactor(DriveConstants.ENCODER_TO_METERS);
        encoder.setVelocityConversionFactor(DriveConstants.ENCODER_TO_METERS);

        return encoder;
    }

    @Override
    public void periodic() {
        // odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(),
        //         rightEncoder.getPosition());
        // SmartDashboard.putNumber("Drivetrain/Power", getLeftPower());
        // SmartDashboard.putNumber("Drivetrain/Pitch", getPitch());
        // SmartDashboard.putNumber("Drivetrain/Yaw", getYaw());
        // SmartDashboard.putNumber("Drivetrain/Roll", getRoll());
        // SmartDashboard.putNumber("Drivetrain/GyroHeadin", gyro.getRotation2d().getDegrees());
        // SmartDashboard.putString("Drivetrain/Pose", getPose().toString());
        


        if (Constants.DIAGNOSTICS) {
            SmartDashboard.putNumber("Drivetrain/Power", getLeftPower());
            SmartDashboard.putNumber("Drivetrain/Pitch", getPitch());
            SmartDashboard.putNumber("Drivetrain/Yaw", getYaw());
            SmartDashboard.putNumber("Drivetrain/Roll", getRoll());
            SmartDashboard.putNumber("Drivetrain/GyroHeadin", gyro.getRotation2d().getDegrees());
            SmartDashboard.putString("Drivetrain/Pose", getPose().toString());
            SmartDashboard.putNumber("Drivetrain/Left Position", getLeftPosition());
            SmartDashboard.putNumber("Drivetrain/Right Position", getRightPosition());

            SmartDashboard.putNumber("Drivetrain/Left Velocity", getLeftVelocity());
            SmartDashboard.putNumber("Drivetrain/Right Velocity", getRightVelocity());

            SmartDashboard.putNumber("Drivetrain/Left Temp", leftLeader.getMotorTemperature());
            SmartDashboard.putNumber("Drivetrain/Right Temp", rightLeader.getMotorTemperature());

            SmartDashboard.putNumber("Drivetrain/Left Current", leftLeader.getOutputCurrent());
            SmartDashboard.putNumber("Drivetrain/Right Current", rightLeader.getOutputCurrent());

            SmartDashboard.putNumber("Drivetrain/Left Volts", leftLeader.getAppliedOutput());
            SmartDashboard.putNumber("Drivetrain/Right Volts", rightLeader.getAppliedOutput());


            SmartDashboard.putNumber("Drivetrain/Yaw", gyro.getYaw());
        }
    }

}