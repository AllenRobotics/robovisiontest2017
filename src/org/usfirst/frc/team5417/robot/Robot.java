package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.usfirst.frc.team5417.cv2017.ChannelRange;
import org.usfirst.frc.team5417.cv2017.ComputerVision2017;
import org.usfirst.frc.team5417.cv2017.ComputerVisionResult;
import org.usfirst.frc.team5417.cv2017.opencvops.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	private CameraServer cameraServer;
	private CameraReader cameraReader;
	private CvSource computerVisionOutputStream;

	// Real images
	ChannelRange hueRange = new ChannelRange(150, 200);
	ChannelRange satRange = new ChannelRange(0.2, 1.0);
	ChannelRange valRange = new ChannelRange(180, 256);

	List<BooleanMatrix> horizontalTemplates = new ArrayList<BooleanMatrix>();
	List<BooleanMatrix> verticalTemplates = new ArrayList<BooleanMatrix>();

	int dilateErodeKernelSize = 3;
	int removeGroupsSmallerThan = 12;
	int numberOfScaleFactors = 20;
	double minimumTemplateMatchPercentage = 0.7;

	// dummy PID that wants to approach 4 feet
	// private PID distancePID = new PID(1, 20000, 0, PIDSourceType.kRate, 4);

	double[] gearLookUpTable = { 250, // 0 feet
			104, // 1 foot
			53.7, 37.5, 28, 22.5, 19, 16.4, 14.2, 12.4, 11.6, 10.6, // 11 feet
			0 // BEYOND
	};

	RobotDrive myRobot = new RobotDrive(0, 1); // class that handles basic drive
												// operations
	Joystick leftStick = new Joystick(0); // set to ID 1 in DriverStation
	Joystick rightStick = new Joystick(1); // set to ID 2 in DriverStation

	public Robot() {
		super();
		// stupid high expiration of 10 seconds
		myRobot.setExpiration(10);

		cameraServer = CameraServer.getInstance();
		cameraServer.startAutomaticCapture();// "cam0", 0);

		computerVisionOutputStream = CameraServer.getInstance().putVideo("CV2017", 320, 240);

		// String camera = "cam0";

		// cameraReader = new CameraReader(camera);
		cameraReader = new CameraReader();

		// horizontalTemplates.add(new BooleanMatrix(40, 150, true));
		// horizontalTemplates.add(new BooleanMatrix(20, 150, true));
		horizontalTemplates.add(new BooleanMatrix(20, 75, true));
		horizontalTemplates.add(new BooleanMatrix(10, 75, true));

		// verticalTemplates.add(new BooleanMatrix(150, 60, true));
		verticalTemplates.add(new BooleanMatrix(75, 30, true));
	}

	/**
	 * Runs the motors with tank steering.
	 */
	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			myRobot.tankDrive(leftStick, rightStick);

			// Mat m = this.cameraReader.read();
			// computerVisionOutputStream.putFrame(m);

			ComputerVision2017 gearCV2017 = new ComputerVision2017();
			ComputerVisionResult cvResult = gearCV2017.DoComputerVision(this.cameraReader, 320, hueRange, satRange,
					valRange, dilateErodeKernelSize, removeGroupsSmallerThan, numberOfScaleFactors,
					minimumTemplateMatchPercentage, verticalTemplates, gearLookUpTable);

			SmartDashboard.putNumber("CV distance", cvResult.distance);

			if (cvResult.visionResult != null) {
				Mat euc3 = new Mat();
				cvResult.visionResult.assignTo(euc3, CvType.CV_8UC3);
				computerVisionOutputStream.putFrame(euc3);

				cvResult.visionResult.release();
				euc3.release();
			}

			Timer.delay(0.005); // wait for a motor update time
		}
	}
}
