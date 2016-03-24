package TT;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.sql.Blob;
import java.sql.Time;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.management.timer.Timer;

import java.lang.Double;
import java.lang.reflect.Array;
import java.lang.Boolean;

//Serial
//import jssc.SerialPort;
//import jssc.SerialPortException;

//Ethernet
//import java.io.*;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import sun.nio.ch.Net;

/**
 * 
 * @author Elijah Kaufman
 * @version 1.0
 * @description Uses opencv and network table 3.0 to detect the vision targets
 *
 */
public class TowerTracker {

	/**
	 * static method to load opencv and networkTables
	 */
	// initialize opencv
	static {

		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		// NetworkTable.setClientMode();
		// NetworkTable.setIPAddress("roborio-1806.local");
	}

	// constants for the color rbg values
	public static final Scalar RED = new Scalar(0, 0, 255), BLUE = new Scalar(255, 0, 0), GREEN = new Scalar(0, 255, 0),
			BLACK = new Scalar(0, 0, 0), YELLOW = new Scalar(0, 255, 255), WHITE = new Scalar(255, 255, 255);
	// these are the threshold values in order
	public static Scalar LOWER_BOUNDS = new Scalar(58, 0, 109), UPPER_BOUNDS = new Scalar(93, 255, 240);

	// the size for resing the image
	public static final Size resize = new Size(320, 240);
	public static int loops = 0;
	public static long lastTime = 0;

	// ignore these
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;

	public static ServerThread st;

	// Constants for known variables
	// the height to the top of the target in first stronghold is 97 inches
	public static final int TOP_TARGET_HEIGHT = 97;
	// the physical height of the camera lens
	public static final int TOP_CAMERA_HEIGHT = 40;

	// camera details, can usually be found on the datasheets of the camera
	public static final double VERTICAL_FOV = 51;
	public static final double HORIZONTAL_FOV = 67;
	public static final double CAMERA_ANGLE = 10;

	public static double minHeight = 25;
	public static double minWidth = 25;
	public static double minAspectRatio = 1;
	public static int minHue = 58;
	public static int minSaturation = 0;
	public static int minValue = 109;
	public static int maxHue = 93;
	public static int maxSaturation = 255;
	public static int maxValue = 240;
	public static boolean calibrationMode = true;

	public static boolean shouldRun = true;

	public static double x, y, targetX, targetY, distance, azimuth;
	public static boolean goalFound;

	public static boolean connectedToCamera = false;
	public static boolean connectedToRio = false;

	static DatagramPacket packet;

	public static ByteBuffer buf;
	public static Rect rec;
	public static Point center;
	public static Point centerw;
	
	static NetworkTable table;
	public static int goalFoundInt = 0;
	
	/**
	 * 
	 * @param args
	 *            command line arguments just the main loop for the program and
	 *            the entry points
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		matOriginal = new Mat();
		matHSV = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();
		st = new ServerThread();
		st.start();
		double fps = 0.0;
		double lastfps = 0.0;
		// NetworkTable table = NetworkTable.getTable("SmartDashboard");
		// main loop of the program
		// setup camera
		try {

			System.out.println("TowerTracker loading...");
			System.out.println("Connecting to camera...");

			// opens up the camera stream and tries to load it
			videoCapture = new VideoCapture();
			//this connects to an onboard cam
			//http://roborio-1806-frc.local:5800/?action=stream
			//videoCapture.open("http://10.18.6.16/mjpg/video.mjpg");
			//videoCapture.open("http://roborio-1806-frc.local:5800/stream_simple.html");
			videoCapture.open("http://roborio-1806-frc.local:5800/?action=stream");

			System.out.println("Video capture has been initialized...");
			
			//Wait until videocapture is opened
			while (!videoCapture.isOpened()) {
				
					videoCapture.open("http://roborio-1806-frc.local:5800/?action=stream");
				
			}
			System.out.println("Video stream has been opened...");

			connectedToCamera = true;
			
			
			//NetworkTables sterf
			NetworkTable.setClientMode();
			NetworkTable.setTeam(1806);
			NetworkTable.initialize();
			table = NetworkTable.getTable("TowerTracker");
			//wait for networktables to connect
			while(!table.isConnected()){
				
			}
			

		} catch (Exception e) {
			System.out.println("Failed to connect to camera: " + e);
			connectedToCamera = false;
		}

		// main running loop
		lastTime = System.currentTimeMillis();
		while (shouldRun) {
			try {

				if (!videoCapture.isOpened()) {
					System.out.println("Lost camera connection");
					connectedToCamera = false;
					
					while(!videoCapture.isOpened()){
						//wait
						videoCapture.open("http://roborio-1806-frc.local:5800/?action=stream");
					}
					
				} else {
					connectedToCamera = true;
				}
				
				if(!table.isConnected()){
					while(!table.isConnected()){
						//wait
					}
				}

				if (calibrationMode) {
					loops++;
					if (loops >= 50) {
						loops = 0;
						setParameters();
						// Imgcodecs.imwrite("/home/ubuntu/TowerTracker/output.png",
						// matOriginal);
						Imgcodecs.imwrite("C:/Users/Becker/Desktop/TowerTracker/output.png", matOriginal);
					}
				}

				// time to actually process the acquired images
				processImage();
				if(goalFound){
					goalFoundInt = 1;
				}else{
					goalFoundInt = 0;
				}
				table.putNumber("GoalFound", goalFoundInt);
				table.putNumber("AngleToGoal", azimuth);
				table.putNumber("Distance", distance);
				fps = (1000 / (System.currentTimeMillis() + 1 - lastTime));
				System.out.println("Processing FPS: " + (fps + lastfps) / 2);
				//System.out.println(azimuth);
				lastTime = System.currentTimeMillis();
				lastfps = fps;

				// st.sendInfo(goalFound, azimuth, distance);

			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		// make sure the java process quits when the loop finishes
		videoCapture.release();
		System.exit(0);
	}

	/**
	 * 
	 * reads an image from a live image capture and outputs information to the
	 * SmartDashboard or a file
	 */
	public static void processImage() {
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

		contours.clear();
		// capture from the axis camera
		// videoCapture.read(matOriginal);
		videoCapture.read(matOriginal);
		// captures from a static file for testing
		// matOriginal = Imgcodecs.imread("someFile.png");
		Imgproc.cvtColor(matOriginal, matHSV, Imgproc.COLOR_BGR2HSV);
		Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
		Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		// make sure the contours that are detected are at least 20x20
		// pixels with an area of 400 and an aspect ration greater then 1
		for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
			MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
			rec = Imgproc.boundingRect(matOfPoint);
			if (rec.height < minHeight || rec.width < minWidth) {
				iterator.remove();
				continue;
			}
			float aspect = (float) rec.width / (float) rec.height;
			if (aspect < minAspectRatio)
				iterator.remove();
		}
		for (MatOfPoint mop : contours) {
			rec = Imgproc.boundingRect(mop);
			Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), BLACK);
		}
		// if there is only 1 target, then we have found the target we want
		if (contours.size() == 1) {
			goalFound = true;
			rec = Imgproc.boundingRect(contours.get(0));
			// "fun" math brought to you by miss daisy (team 341)!
			y = rec.br().y + rec.height / 2;
			y = -((2 * (y / matOriginal.height())) - 1);
			distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT)
					/ Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
			// angle to target...would not rely on this
			targetX = rec.tl().x + rec.width / 2.000;
			targetX = (2 * (targetX / matOriginal.width())) - 1.000;
			azimuth = normalize360(targetX * HORIZONTAL_FOV / 2.000 + 0.000);
			// drawing info on target
			center = new Point(rec.br().x - rec.width / 2 - 15, rec.br().y - rec.height / 2);
			centerw = new Point(rec.br().x - rec.width / 2 - 15, rec.br().y - rec.height / 2 - 20);
			// Imgproc.putText(matOriginal, "" + (int) distance, center,
			// Core.FONT_HERSHEY_PLAIN, 1, WHITE);
			Imgproc.putText(matOriginal, "" + (double) azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, WHITE);
		} else {
			goalFound = false;
		}

		// output an image for debugging

		// shouldRun = false;
	}

	/**
	 * @param angle
	 *            a nonnormalized angle
	 */
	public static double normalize360(double angle) {
		while (angle >= 360.0) {
			angle -= 360.000;
		}
		while (angle < 0.0) {
			angle += 360.000;
		}
		return angle;
	}

	public static void setParameters() {
		int iterations = 0;
		try {
			// List<String> lines =
			// Files.readAllLines(Paths.get("/home/ubuntu/TowerTracker/parameters.txt"),
			// StandardCharsets.UTF_8);
			List<String> lines = Files.readAllLines(Paths.get("C:/Users/Becker/Desktop/TowerTracker/parameters.txt"),
					StandardCharsets.UTF_8);
			Iterator<String> iterator = lines.iterator();
			while (iterator.hasNext()) {
				String str = iterator.next();
				if (str.contains("#")) {
					iterator.remove();
				} else {
					if (iterations == 0) {
						// first pass, this value is the height
						minHeight = Double.valueOf(str);
						iterations++;
					} else if (iterations == 1) {
						// second pass, this value is the width
						minWidth = Double.valueOf(str);
						iterations++;
					} else if (iterations == 2) {
						// last pass, this value is aspect ratio
						minAspectRatio = Double.valueOf(str);
						iterations++;
					} else if (iterations == 3) {
						minHue = Integer.valueOf(str);
						iterations++;
					} else if (iterations == 4) {
						minSaturation = Integer.valueOf(str);
						iterations++;
					} else if (iterations == 5) {
						minValue = Integer.valueOf(str);
						iterations++;
					} else if (iterations == 6) {
						maxHue = Integer.valueOf(str);
						iterations++;
					} else if (iterations == 7) {
						maxSaturation = Integer.valueOf(str);
						iterations++;
					} else if (iterations == 8) {
						maxValue = Integer.valueOf(str);
						iterations++;
					} else if (iterations == 9) {
						calibrationMode = Boolean.valueOf(str);
					}

				}
			}

			double[] lower = new double[] { (double) minHue, (double) minSaturation, (double) minValue };
			LOWER_BOUNDS.set(lower);

			double[] upper = new double[] { (double) maxHue, (double) maxSaturation, (double) maxValue };
			UPPER_BOUNDS.set(upper);

			System.out.println("Parameters set.");

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
