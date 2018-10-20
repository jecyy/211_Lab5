package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the light localization
 * @author jecyy
 *
 */
public class LightLocalizer {

	private static float color = 0;
	private static Odometer odo;
	private static final int ROTATE_SPEED = 100;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double leftRadius, rightRadius, track;
	private static double y1, y2, x1, x2; // angle at positive y-axis, negative y-axis, positive x-axis, negative x-axis
	private static final double d = 12.8; // distance between the light sensor and the center of rotation
	private static final double extraDis = 5.0; // extra distance traveled before entering Light Localizer
	public static boolean finished = false;
	
	/**
	 * This is the main method that runs the Light Localizer
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

		// Sleep for 1 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		
		leftMotor = left;
		rightMotor = right;
		leftRadius = leftR;
		rightRadius = rightR;
		track = trac;
		odo = odom;
		
		// turn -135 degree
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(leftRadius, track, -135), true);
		rightMotor.rotate(-convertAngle(rightRadius, track, -135), false);
		
		// go to set-up position
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.backward();

		// detect the first black line and then we can start the Localization algorithm
		while (true) {
			color = LightSensorPoller.get_light();
			
			if ( color < 450 ) { // black line detected
				Sound.beep();
				leftMotor.setSpeed(ROTATE_SPEED); // move backward to start light localization
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertDistance(leftRadius, -extraDis), true);   // travel a short distance
				rightMotor.rotate(convertDistance(rightRadius, -extraDis), false);// for better performance
				
				leftMotor.setSpeed(0); // stop and
				rightMotor.setSpeed(0);// get ready for rotation
				
				break;
			}
		}
		
		// find x1
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
		while(true) {
			color = LightSensorPoller.get_light();
			
			if (color < 450 && odo.getXYT()[2] > 180) {
				Sound.beep();
				x1 = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}
		
		// find y2
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
		while(true) {
			color = LightSensorPoller.get_light();
			
			if (color < 450 && odo.getXYT()[2] > 270) {
				Sound.beep();
				y2 = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}
		
		// find x2
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
		while(true) {
			color = LightSensorPoller.get_light();
			
			if (color < 450 && odo.getXYT()[2] > 90) {
				Sound.beep();
				x2 = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}
		
		// find y1
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
		while(true) {
			color = LightSensorPoller.get_light();
			
			if (color < 450 && odo.getXYT()[2] > 180) {
				Sound.beep();
				y1 = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}
		
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		
		// now we have x1, x2, y1, y2
		// we can calculate the position of the robot
		// using the algorithm provided in the tutorial
		double thetaY = Math.abs(y1 - y2);
		double thetaX = Math.abs(x1 - x2);
		if (thetaY > 180) thetaY -= 180;
		if (thetaX > 180) thetaX -= 180;
		double dx = d * Math.cos(thetaY * Math.PI / 360);
		double dy = d * Math.cos(thetaX * Math.PI / 360);
		double current = odo.getXYT()[2];
		
		// move to the origin
		// head to 90 degree
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(leftRadius, track,  - current + 90), true);
		rightMotor.rotate(-convertAngle(rightRadius, track,  - current + 90), false);
		// move to Y-axis
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, dx), true);
		rightMotor.rotate(convertDistance(rightRadius, dx), false);
		// head to 0 degree
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
		rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
		// move to X-axis
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, dy), true);
		rightMotor.rotate(convertDistance(rightRadius, dy), false);
		
		finished = true;
	}


	// give the light sensor reading
	public static float lightReading() {
		return color;
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	public LightLocalizer() {

	}

}
