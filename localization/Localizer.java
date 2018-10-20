package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This is an alternative approach to do localization,
 * using only an ultrasonic sensor and based on Law of cosines
 * @author jecyy
 *
 */

public class Localizer {
	private static int distance; // distance reading from the ultrasonic sensor
	private static Odometer odo;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double leftRadius, rightRadius, track;
	private static final int ROTATE_SPEED = 125;
	private static final double TILE_SIZE = 30.48;
	private static final int d = 35; // threshold for determining alpha and beta
	private static double gamma = 30; // a fixed distacne that the robot rotates after detecting the wall
	private static double a, b, alpha; // two edges of a triangle, with an angle used for correct orientation
	private static double xdis, ydis; // distances from the wall
	private static final double extraDis = 6.8; // extra distance traveled before entering Light Localizer
	private static final double pi = Math.PI;
	public static boolean finished = false; // indicates whether the whole ultrasonic localization process is finished


	/**
	 * This is the main method that runs the Ultrasonic Localizer
	 * @param left
	 * @param right
	 * @param leftR
	 * @param rightR
	 * @param trac
	 * @param odom
	 */
	public static void run(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right,
			double leftR, double rightR, double trac, Odometer odom) {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {left, right}) {
			motor.stop();
			motor.setAcceleration(6000);
		}

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}

		distance = UltrasonicPoller.get_distance();

		odo = odom;
		leftMotor = left;
		rightMotor = right;
		leftRadius = leftR;
		rightRadius = rightR;
		track = trac;

		// determine the starting position of the robot, 
		// if it is facing the wall, make it rotate
		while (distance < 50) {
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			} 
			turn(90);
			distance = UltrasonicPoller.get_distance();
		}
		freeze(); // stop the robot once facing away the wall

		// do localization

		// 1.calculate distance from  left wall
		keepTurning(false); // rotate anti-clockwise
		while (distance > d) {
			distance = UltrasonicPoller.get_distance();
		}
		Sound.beep();
		freeze();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		a = UltrasonicPoller.get_distance() + extraDis;
		turn(-gamma);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		b = UltrasonicPoller.get_distance() + extraDis;
		xdis = a * b * Math.sin(gamma * pi / 180) / 
				(Math.sqrt(a*a + b*b - 2*a*b*Math.cos(gamma*pi/180)));
		// 2. calculate distance from back wall
		turn(90); // first turn a fixed distance to avoid mis-detecting
		distance = 200; // initial condition to enter while loop
		keepTurning(true); // rotate clockwise
		while (distance > d) {
			distance = UltrasonicPoller.get_distance();
		}
		Sound.beep();
		freeze();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		a = UltrasonicPoller.get_distance() + extraDis;
		turn(gamma);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		b = UltrasonicPoller.get_distance() + extraDis;
		ydis = a * b * Math.sin(gamma * pi / 180) / 
				(Math.sqrt(a*a + b*b - 2*a*b*Math.cos(gamma*pi/180)));
		// 3. calculate xdis again
		turn(-90); // first turn a fixed distance to avoid mis-detecting
		distance = 200;
		keepTurning(false); // rotate anti-clockwise
		while (distance > d) {
			distance = UltrasonicPoller.get_distance();
		}
		Sound.beep();
		freeze();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
		a = UltrasonicPoller.get_distance() + extraDis;
		turn(-gamma);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		b = UltrasonicPoller.get_distance() + extraDis;
		xdis += a * b * Math.sin(gamma * pi / 180) / 
				(Math.sqrt(a*a + b*b - 2*a*b*Math.cos(gamma*pi/180)));
		xdis /= 2;
		odo.setX(xdis);
		// 4. calculate ydis again
		turn(90); // first turn a fixed distance to avoid mis-detecting
		distance = 200; // initial condition to enter while loop
		keepTurning(true);
		while (distance > d) {
			distance = UltrasonicPoller.get_distance();
		}
		Sound.beep();
		freeze();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		a = UltrasonicPoller.get_distance() + extraDis;
		turn(gamma);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		} 
		b = UltrasonicPoller.get_distance() + extraDis;
		ydis = ydis + a * b * Math.sin(gamma * pi / 180) / 
				(Math.sqrt(a*a + b*b - 2*a*b*Math.cos(gamma*pi/180)));
		ydis /= 2;
		odo.setY(ydis);

		// now we have a corrected x and y, what's left is to correct theta
		freeze();
		alpha = Math.acos(ydis / b) * 180 / pi; // the angle between the current orientation and 180 degree

		odo.setTheta(180 + alpha);
		
		travelToStart();
		turn(-odo.getXYT()[2]);
		freeze();

		finished = true; // localization finished
	}

	private static void turn (double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
		rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);
	}

	/**
	 * This method keeps the robot rotating with a choice of clockwise or anti
	 * @param isClockwise
	 */
	private static void keepTurning (boolean isClockwise) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if (isClockwise == true) {
			leftMotor.forward();
			rightMotor.backward();
		}
		else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}

	private static void freeze() {
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}

	private static void travelToStart() {
		double deltaX = TILE_SIZE - odo.getXYT()[0];
		double deltaY = TILE_SIZE - odo.getXYT()[1];
		double toTravel = Math.sqrt(deltaX * deltaX + deltaY * deltaY); // calculate the distance between the current and next way point

		// here we are trying to find theta based on different cases,
		// where theta is the angle that the robot needs to rotate
		double currentT = odo.getXYT()[2];
		double theta;
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

		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		leftMotor.rotate(convertDistance(leftRadius, toTravel), true);
		rightMotor.rotate(convertDistance(rightRadius, toTravel), false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public Localizer() {

	}
}
