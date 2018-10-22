package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.localization.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class lets the robot travel to search area and do searching
 * @author jecyy
 *
 */
public class Search {
	private static int LLX = 3, LLY = 3, 
			           URX = 7, URY = 7; // coordinates of the Lower-Left and Upper-Right hand corner of the search region
	private static int TR = 0; // the number defining the color of the target ring
	private static final int SC = 0; // the starting corner
	private static double Rmin, Rmax,
	                      Gmin, Gmax,
	                      Bmin, Bmax; // range for detection
	private static EV3LargeRegulatedMotor left, right;
	private static double leftR, rightR, trac;
	private static Odometer odo;
	private static double deltaX, deltaY;
	private static double toTravel, currentT, theta;
	private static double pi = Math.PI;
	private static int FORWARD_SPEED = 175, ROTATE_SPEED = 100;
	private static boolean found = false; // indicates if target ring is found
	private static final double ts = 30.48; // TILE SIZE


	public static void run(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double track, Odometer odometer) {

		left = leftMotor;
		right = rightMotor;
		odo = odometer;
		leftR = leftRadius;
		rightR = rightRadius;
		trac = track;
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(1000);
		}

		// prepare odometer
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {
		}

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		
		// adjust odometer
		odo.setX(ts);
		odo.setY(ts);
		odo.setTheta(0);

		// set the color range
		if (TR == 0) { // blue

		}
		else if (TR == 1) { // green

		}
		else if (TR == 2) { // yellow

		}
		else { // orange

		}


		travelTo(LLX, LLY); // travel to the lower left corner
		Sound.beep();       // and BEEP
		// adjust the odometer
		odo.setX(3 * ts);
		odo.setY(3 * ts);
		odo.setTheta(45);

		for (int i = 1; i <= URX - LLX - 1; i++) {
			travelTo(LLX + i, LLY); // travel a tile, and ensure that the robot is heading positive X-axis
			searchLine();
			left.rotate(-720, true);
			right.rotate(-720, false);
			travelTo(LLX + i, LLY); // go back
			Sound.beepSequenceUp();
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
			if (found == true) break;
		}

		travelTo(URX, LLY);
		travelTo(URX, URY);

		if (found == false) {
			for (int i = 1; i <= URX - LLX - 1; i++) {
				travelTo(URX - i, URY);
				searchLine();
				left.rotate(-720, true);
				right.rotate(-720, false);
				travelTo(URX - i, URY);
				Sound.beepSequenceUp();
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
				}
				if (found == true) break;
			}
			travelTo(URX, URY);
		}

		left.setSpeed(0);
		right.setSpeed(0);


	}


	public static int findMatch() {
		double  r = ColorSensorPoller.getR(),
				g = ColorSensorPoller.getG(),
				b = ColorSensorPoller.getB();

		if (r < b) // blue
			return 0;
		else if (r < g) // green
			return 1; 
		else if (r > 2 * g) return 3; // orange
		else return 2; // yellow
		
	}

	private static void searchLine() {
		turn(-90); // turn to positive Y-axis
		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		int filterCount = 0;

		while(
				(odo.getXYT()[1] <= (URY + 0.5) * ts) && (odo.getXYT()[1] >= (LLY - 0.5) * ts)
				) {
			left.forward();
			right.forward();
			if (UltrasonicPoller.get_distance() > 4.5) filterCount = 0;
			else filterCount ++;
			if (filterCount >= 3) break;
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		};

		left.setSpeed(0);
		right.setSpeed(0);

		if (UltrasonicPoller.get_distance() <= 4.5) {

			if (findMatch() == SC) {
				Sound.beep();
				found = true;
			}
			else {
				Sound.twoBeeps();
			}
		}
	}

	private static void travelTo(int x, int y) {
		deltaX = x * ts - odo.getXYT()[0];
		deltaY = y * ts - odo.getXYT()[1];
		toTravel = Math.sqrt(deltaX * deltaX + deltaY * deltaY); // calculate the distance between the current and next way point

		// here we are trying to find theta based on different cases,
		// where theta is the angle that the robot needs to rotate
		currentT = odo.getXYT()[2];
		if (deltaX == 0 && deltaY >= 0) {
			theta = - currentT;
		}
		else if (deltaX == 0) {
			theta = 180 - currentT;
		}
		else if (deltaY == 0 && deltaX < 0) {
			theta = - currentT - 90;
		}
		else if (deltaY == 0 && deltaX > 0) {
			theta = - currentT + 90;
		}
		else {
			theta = 90 - Math.atan(deltaY / deltaX) * 180 / pi - currentT;
			if (deltaX <= 0 && deltaY <= 0) {
				theta = theta - 180;
			}
			if (deltaX <= 0 && deltaY >= 0) {
				theta = - theta;
			}
		}
		if (theta >= 180) theta -= 360;
		if (theta <= -180) theta += 360;

		// take the turn
		turn(theta);

		// drive forward

		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		left.rotate(convertDistance(leftR, toTravel), true);
		right.rotate(convertDistance(rightR, toTravel), false);
	}

	private static void turn (double theta) {
		left.setSpeed(ROTATE_SPEED);
		right.setSpeed(ROTATE_SPEED);

		left.rotate(convertAngle(leftR, trac, theta), true);
		right.rotate(-convertAngle(rightR, trac, theta), false);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (pi * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, pi * width * angle / 360.0);
	}

	private static double[] normalize(double R, double G, double B) {
		double normal[] = new double[3];
		normal[0] = R / Math.sqrt(R * R + G * G + B * B);
		normal[1] = G / Math.sqrt(R * R + G * G + B * B);
		normal[2] = B / Math.sqrt(R * R + G * G + B * B);
		return normal;
	}
}
