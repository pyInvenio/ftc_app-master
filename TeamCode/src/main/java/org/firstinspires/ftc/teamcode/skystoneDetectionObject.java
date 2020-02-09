package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class skystoneDetectionObject {
	private static int valMid = -1;
	private static int valLeft = -1;
	private static int valRight = -1;
	private int[] values = {valLeft, valMid, valRight};
	private static float rectHeight = .6f/8f;
	private static float rectWidth = 2f/8f;

	private static float offsetX = .75f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
	private static float offsetY = 2.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

	private static float[] midPos = {4f/8f, 4f/8f+offsetY};//0 = col, 1 = row
	private static float[] leftPos = {2f/8f-offsetX, 4f/8f+offsetY};
	private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
	//moves all rectangles right or left by amount. units are in ratio to monitor

	private final int rows = 640;
	private final int cols = 480;

	OpenCvCamera phoneCam;
	public skystoneDetectionObject(){
	}
	public void camSetup (HardwareMap hwMap) {
		int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
		phoneCam.openCameraDevice();//open camera
		phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
		phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
	}
	public int[] getValues() {

		return values;
	}
	public void updateValues() {
		values[0] = valLeft;
		values[1] = valMid;
		values[2] = valRight;

	}
	//detection pipeline
	public class StageSwitchingPipeline extends OpenCvPipeline
	{
		Mat yCbCrChan2Mat = new Mat();
		Mat thresholdMat = new Mat();
		Mat all = new Mat();
		List<MatOfPoint> contoursList = new ArrayList<>();

		@Override
		public Mat processFrame(Mat input)
		{
			contoursList.clear();

			//color diff cb.
			//lower cb = more blue = skystone = white
			//higher cb = less blue = yellow stone = grey
			Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
			Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

			//b&w
			Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
			//outline/contour
			Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
			yCbCrChan2Mat.copyTo(all);//copies mat object
			Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours

			updateValues(input);

			//create three points
			Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
			Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
			Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

			//draw circles on those points
			Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
			Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
			Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

			//draw 3 rectangles
			Imgproc.rectangle(//1-3
					all,
					new Point(
							input.cols()*(leftPos[0]-rectWidth/2),
							input.rows()*(leftPos[1]-rectHeight/2)),
					new Point(
							input.cols()*(leftPos[0]+rectWidth/2),
							input.rows()*(leftPos[1]+rectHeight/2)),
					new Scalar(0, 255, 0), 3);
			Imgproc.rectangle(//3-5
					all,
					new Point(
							input.cols()*(midPos[0]-rectWidth/2),
							input.rows()*(midPos[1]-rectHeight/2)),
					new Point(
							input.cols()*(midPos[0]+rectWidth/2),
							input.rows()*(midPos[1]+rectHeight/2)),
					new Scalar(0, 255, 0), 3);
			Imgproc.rectangle(//5-7
					all,
					new Point(
							input.cols()*(rightPos[0]-rectWidth/2),
							input.rows()*(rightPos[1]-rectHeight/2)),
					new Point(
							input.cols()*(rightPos[0]+rectWidth/2),
							input.rows()*(rightPos[1]+rectHeight/2)),
					new Scalar(0, 255, 0), 3);

			return all;
		}

		public void updateValues(Mat input) {
			//get values from frame
			double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
			valMid = (int)pixMid[0];

			double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
			valLeft = (int)pixLeft[0];

			double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
			valRight = (int)pixRight[0];

		}

	}
	public int getSkyStonePosition(int[] vals){
		if(vals[0]==0)
			return 1;
		else if(vals[1] ==0)
			return 2;
		return 3;
	}
}
