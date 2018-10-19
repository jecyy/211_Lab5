package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class runs the Ultrasonic Localizer.
 * @author jealo
 *
 */
public class UltrasonicLocalizer {

	private static int distance; // distance reading from the ultrasonic sensor
	private static Odometer odo;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double leftRadius, rightRadius, track;
	private static final int ROTATE_SPEED = 100;
	private static final double TILE_SIZE = 30.48;
	private static final int d = 28; // threshold for determining alpha and beta
	private static final int k = 1;  // tolerance of threshold
	private static double alpha; // back wall angle
	private static double beta; // left angle
	private static double deltaT; // angle to be used for correction
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
		// if it is facing the wall, do rising edge, otherwise falling edge
		if (distance > TILE_SIZE) {
			fallingEdge();
		}
		else {
			risingEdge();
		}
		
		// calculate the angle for localization rotation
		if (alpha < beta) {
			deltaT = 45 - (alpha + beta) / 2 + 180;
		}
		else {
			deltaT = 225 - (alpha + beta) / 2 + 180;
		}
		double diff = 360 - odo.getXYT()[2] - deltaT; // difference between the real 0 degree orientation
													  // and the current heading
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(leftRadius, track, diff), true);
		rightMotor.rotate(-convertAngle(rightRadius, track, diff), false);
		odo.setTheta(0); // now the robot should face the 0 degree orientation
						 // reset the odometer reading for correction
		finished = true; // ultrasonic localization finished
	}
	
	/**
	 * This method implements the falling edge localization
	 */
	private static void fallingEdge() {
		// start clockwise rotation
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
		
		// search for the back wall
		while(true) {
			distance = UltrasonicPoller.get_distance();
			if (distance < d - k) {
				Sound.beep();
				alpha = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}
		
		// change rotation direction
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.forward();
	    try {
		      Thread.sleep(1500);
		    } catch (InterruptedException e) {
		    	
		    }
		
	    // search for the left wall
		while(true) {
			distance = UltrasonicPoller.get_distance();
			if (distance < d - k) {
				Sound.beep();
				beta = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}    
	}

	/**
	 * This method implements the rising edge localization
	 */
	private static void risingEdge() {
		// start anti-clockwise rotation
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.forward();
		
		// search for the back wall
		while(true) {
			distance = UltrasonicPoller.get_distance();
			if (distance > d - k) {
				Sound.beep();
				alpha = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}
		
		// change rotation direction
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();
	    try {
		      Thread.sleep(1500);
		    } catch (InterruptedException e) {
		    	
		    }
		
	    // search for the left wall
		while(true) {
			distance = UltrasonicPoller.get_distance();
			if (distance > d - k) {
				Sound.beep();
				beta = odo.getXYT()[2];
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
		}  
	}


	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public UltrasonicLocalizer() {

	}
}
