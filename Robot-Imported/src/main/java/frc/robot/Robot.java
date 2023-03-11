package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Robot extends TimedRobot {
    Thread m_visionThread;
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // m_visionThread =
        //     new Thread(
        //         () -> {
        //         UsbCamera camera = CameraServer.startAutomaticCapture();
        //         camera.setResolution(640, 480);

        //         CvSink cvSink = CameraServer.getVideo();
        //         CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

        //         Mat mat = new Mat();

        //         while (!Thread.interrupted()) {
        //             if (cvSink.grabFrame(mat) == 0) {
        //             // Send the output the error.
        //             outputStream.notifyError(cvSink.getError());
        //             // skip the rest of the current iteration
        //             continue;
        //             }
        //             // Put a rectangle on the image
        //             Imgproc.rectangle(
        //                 mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
        //             // Give the output stream a new image to display
        //             outputStream.putFrame(mat);
        //         }
        //     });
        // m_visionThread.setDaemon(true);
        // m_visionThread.start();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.diagnostics();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        SmartDashboard.putString("AutoCommand", String.valueOf(autonomousCommand));

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.diagnostics();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.diagnostics();
        if (Constants.DIAGNOSTICS) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
