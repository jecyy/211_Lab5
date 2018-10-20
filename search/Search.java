package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Search {
	private static int LLX = 3, LLY = 3, 
			           URX = 7, URY = 7; // coordinates of the Lower-Left and Upper-Right hand corner of the search region
	private static int TR = 1; // the number defining the color of the target ring
	private static int SC = 0; // the starting corner
	private static double Rmin, Rmax,
	                      Gmin, Gmax,
	                      Bmin, Bmax; // range for detection
	private static EV3LargeRegulatedMotor left, right;
	private static double leftR, rightR, trac;
	private static Odometer odo;
	private static double deltaX, deltaY;
	private static double toTravel, currentT, theta;
	private static double pi = Math.PI;
	private static int FORWARD_SPEED = 150, ROTATE_SPEED = 50;
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
			motor.setAcceleration(300);
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
        
		travelTo(LLX, LLY); // travel to the lower left corner
		Sound.beep();       // and BEEP
		travelTo(LLX + 0.5, LLY + 0.5); // travel half of a tile, and ensure that the robot is heading positive X-axis
        searchLine();
        
        

		leftMotor.stop();
		rightMotor.stop();
	}
	
	private static int findMatch() {
		return 0;
	}
	
	private static void searchLine() {
        turn(-90); // turn to positive Y-axis
        left.setSpeed(FORWARD_SPEED);
        right.setSpeed(FORWARD_SPEED);
        left.forward();
        right.forward();
        
        while(true) {
        	
        }
	}
	
	private static int seeRing() {
		return 0;
	}

	private static void travelTo(double x, double y) {
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
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static double[] normalize(double R, double G, double B) {
		double normal[] = new double[3];
		normal[0] = R / Math.sqrt(R * R + G * G + B * B);
		normal[1] = G / Math.sqrt(R * R + G * G + B * B);
		normal[2] = B / Math.sqrt(R * R + G * G + B * B);
		return normal;
	}
}
