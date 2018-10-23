package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.localization.LightSensorPoller;
import ca.mcgill.ecse211.localization.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class lets the robot travel to search area and do searching
 * 
 * @author jecyy
 *
 */
public class Search {
	private static int LLX = 3, LLY = 3, URX = 7, URY = 7; // coordinates of the Lower-Left and Upper-Right hand corner
															// of the search region
	private static final int TR = 0; // the number defining the color of the target ring
	private static final int SC = 0; // the starting corner
	private static EV3LargeRegulatedMotor left, right;
	private static double leftR, rightR, trac;
	private static Odometer odo;
	private static double deltaX, deltaY;
	private static double toTravel, currentT, theta;
	private static double pi = Math.PI;
	private static int FORWARD_SPEED = 175, ROTATE_SPEED = 100;
	private static boolean found = false; // indicates if target ring is found
	private static final double ts = 30.48; // TILE SIZE
	private static final double offset = 12.50; // vertical distance from light sensor to center of rotation
	private static int county = 0;

	public static void run(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double track, Odometer odometer) {
		left = leftMotor;
		right = rightMotor;
		odo = odometer;
		leftR = leftRadius;
		rightR = rightRadius;
		trac = track;
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
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
		travelToInitialPos(LLX, LLY); // travel to the lower left corner
		Sound.beep(); // and BEEP
		county = LLY; // update the global counts
		turn(-90);
		goUp();
		left.setSpeed(0);
		right.setSpeed(0);
		turn(90);
		linearTravel();
		odo.setX((URX - 1) * ts + offset);
		left.setSpeed(0);
		right.setSpeed(0);
		turn(-90); // after searching an edge
		if (URX - LLX == 3) { // if 3 * 3 grid, we are finished
		} else { // if other cases, keep searching
			goUp();
			goUp();
			left.setSpeed(0);
			right.setSpeed(0); // go 2 grids along positive Y-axis
			turn(-90);
			linearTravel();
			odo.setX((LLX + 1) * ts - offset);
			left.setSpeed(0);
			right.setSpeed(0);
			turn(90);
			goUp();
			if (URX - URY == 6) { // if 6 * 6, search one more edge
				goUp();
				linearTravel();
				odo.setX((URX - 1) * ts + offset);
			}
		}
		travelTo(URX, URY);
	}

	/**
	 * go along positive Y-axis for one tile size (no stopping)
	 */
	private static void goUp() {
		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		left.rotate(convertDistance(2.2, 10), true);
		right.rotate(convertDistance(2.2, 10), false); // to avoid mis-detection of black line
		left.forward();
		right.forward();
		while (LightSensorPoller.get_light() >= 400) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		}
		county++;
		odo.setY(county * ts + offset);
	}

	/**
	 * search along an edge
	 */
	private static void linearTravel() {
		left.rotate(-convertDistance(2.2, 5), true);
		right.rotate(-convertDistance(2.2, 5), false);
		int counter = 0; // to count the number of searches along an edge
		while (counter < URX - LLX - 1) {
			left.setSpeed(FORWARD_SPEED);
			right.setSpeed(FORWARD_SPEED);
			left.forward();
			right.forward();
			if (LightSensorPoller.get_light() < 400) { // black line detected
				searchRound();
				counter++;
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		}
		while (LightSensorPoller.get_light() >= 400) {
			left.setSpeed(FORWARD_SPEED);
			right.setSpeed(FORWARD_SPEED);
			left.forward();
			right.forward();
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		}
	}

	/**
	 * turn around to search nearby rings
	 */
	private static void searchRound() {
		// TODO: change this method
		turn(-40);
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
		}
		double orix = odo.getXYT()[0];
		double oriy = odo.getXYT()[1];
		double curx, cury;
		if (UltrasonicPoller.get_distance() <= ts + 10) { // when see a ring
			while (UltrasonicPoller.get_distance() > 4.5) {
				left.forward();
				right.forward();
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}
			}
			left.setSpeed(0);
			right.setSpeed(0);
			curx = odo.getXYT()[0];
			cury = odo.getXYT()[1];
			if (findMatch() == TR) {
				Sound.beep();
				found = true;
			} else {
				Sound.twoBeeps();
			}
			left.setSpeed(ROTATE_SPEED);
			right.setSpeed(ROTATE_SPEED);
			left.rotate(-convertDistance(2.2, Math.sqrt((curx - orix) * (curx - orix) + (cury - oriy) * (cury - oriy))),
					true);
			right.rotate(
					-convertDistance(2.2, Math.sqrt((curx - orix) * (curx - orix) + (cury - oriy) * (cury - oriy))),
					false);
		}
		turn(80);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		orix = odo.getXYT()[0];
		oriy = odo.getXYT()[1];
		if (UltrasonicPoller.get_distance() <= ts + 10) { // when see a ring
			while (UltrasonicPoller.get_distance() > 4.5) {
				left.forward();
				right.forward();
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
				}
			}
			left.setSpeed(0);
			right.setSpeed(0);
			curx = odo.getXYT()[0];
			cury = odo.getXYT()[1];
			if (findMatch() == TR) {
				Sound.beep();
				found = true;
			} else {
				Sound.twoBeeps();
			}
			left.setSpeed(ROTATE_SPEED);
			right.setSpeed(ROTATE_SPEED);
			left.rotate(-convertDistance(2.2, Math.sqrt(curx - orix) * (curx - orix) + (cury - oriy) * (cury - oriy)),
					true);
			right.rotate(-convertDistance(2.2, Math.sqrt(curx - orix) * (curx - orix) + (cury - oriy) * (cury - oriy)),
					false);
		}
		turn(-40);
		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		left.rotate(convertDistance(2.2, 10), true);
		right.rotate(convertDistance(2.2, 10), false);
	}

	public static int findMatch() {
		double r = ColorSensorPoller.getR(), g = ColorSensorPoller.getG(), b = ColorSensorPoller.getB();
		if (r < b) // blue
			return 0;
		else if (r < g) // green
			return 1;
		else if (r > 2 * g)
			return 3; // orange
		else
			return 2; // yellow

	}

	/**
	 * travel to (LLX, LLY) in order to start search
	 * 
	 * @param x
	 * @param y
	 */
	private static void travelToInitialPos(int x, int y) {
		int ynew = y - 1;
		int xnew = x - 1;
		int xcur = 0;
		int ycur = 0;
		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		left.forward();
		right.forward();
		while (ycur <= ynew) {
			if (LightSensorPoller.get_light() < 400) {// black line detected
				ycur++;
				// Sound.beep();
			}
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
			}
		}
		odo.setY(ycur * ts + offset);
		turn(90);
		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		left.forward();
		right.forward();
		while (xcur <= xnew) {
			if (LightSensorPoller.get_light() < 400) {// black line detected
				xcur++;
				// Sound.beep();
			}
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
			}
		}
		odo.setX(xcur * ts + offset);
	}

	/**
	 * travel to a given grid point
	 * 
	 * @param x
	 * @param y
	 */
	private static void travelTo(double x, double y) {
		deltaX = x * ts - odo.getXYT()[0];
		deltaY = y * ts - odo.getXYT()[1];
		toTravel = Math.sqrt(deltaX * deltaX + deltaY * deltaY); // distance between the current and next point
		// here we are trying to find theta based on different cases,
		// where theta is the angle that the robot needs to rotate
		currentT = odo.getXYT()[2];
		if (deltaX == 0 && deltaY >= 0) {
			theta = -currentT;
		} else if (deltaX == 0) {
			theta = 180 - currentT;
		} else if (deltaY == 0 && deltaX < 0) {
			theta = -currentT - 90;
		} else if (deltaY == 0 && deltaX > 0) {
			theta = -currentT + 90;
		} else {
			theta = 90 - Math.atan(deltaY / deltaX) * 180 / pi - currentT;
			if (deltaX <= 0 && deltaY <= 0) {
				theta = theta - 180;
			}
			if (deltaX <= 0 && deltaY >= 0) {
				theta = -theta;
			}
		}
		if (theta >= 180)
			theta -= 360;
		if (theta <= -180)
			theta += 360;
		// take the turn
		turn(theta);
		// drive forward
		left.setSpeed(FORWARD_SPEED);
		right.setSpeed(FORWARD_SPEED);
		left.rotate(convertDistance(leftR, toTravel), true);
		right.rotate(convertDistance(rightR, toTravel), false);
	}

	/**
	 * turn a given degree (clockwise)
	 * 
	 * @param theta
	 */
	private static void turn(double theta) {
		left.setSpeed(ROTATE_SPEED);
		right.setSpeed(ROTATE_SPEED);
		left.rotate(convertAngle(leftR, trac, theta), true);
		right.rotate(-convertAngle(rightR, trac, theta), false);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
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
}
