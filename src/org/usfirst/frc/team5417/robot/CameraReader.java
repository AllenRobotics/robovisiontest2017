package org.usfirst.frc.team5417.robot;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.usfirst.frc.team5417.cv2017.ImageReader;

import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraReader implements ImageReader {

	String camera;
	private CameraServer cameraServer;
	private CvSink visionCameraSink;

	private Mat frame = new Mat();
	
	public CameraReader() {
		//this.camera = camera;
		cameraServer = CameraServer.getInstance();
		visionCameraSink = cameraServer.getVideo();
	}

	@Override
	public Mat read() {
		visionCameraSink.grabFrame(frame);
		
		Mat f32f = new Mat();
		frame.assignTo(f32f, CvType.CV_32FC3);
		frame.release();
		
		return f32f;
	}

}
