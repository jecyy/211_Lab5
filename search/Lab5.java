package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.localization.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the main class that runs the whole Localization program.
 * @author jealo
 *
 */
public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port portLight = LocalEV3.get().getPort("S2");
	private static final Port portColor = LocalEV3.get().getPort("S3");
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 13.1;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display display = new Display(lcd);
		// Ultrasonic sensor
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// Light sensor
		@SuppressWarnings("resource")
		SensorModes myLight = new EV3ColorSensor(portLight);
		SampleProvider myLightSample = myLight.getMode("Red");
		float[] sampleLight = new float[myLight.sampleSize()];
		// Color sensor
		@SuppressWarnings("resource")
		SensorModes myColor = new EV3ColorSensor(portColor);
		SampleProvider myColorSample = myColor.getMode("RGB");
		float[] sampleColor = new float[3];
		// returned
		UltrasonicPoller ultrasonic = new UltrasonicPoller(usDistance, usData);
		LightSensorPoller light = new LightSensorPoller(myLightSample, sampleLight);
		ColorSensorPoller color = new ColorSensorPoller(myColorSample, sampleColor);


		do {
			// clear the display
			lcd.clear();

			// wait to start
			lcd.drawString("< press R to start >", 0, 0);

			buttonChoice = Button.waitForAnyPress(); // press right button to start
		} while (buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_RIGHT) {
			// Start odometer
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			display.run();

			// Start UltrasonicPoller
			Thread UltrasonicThread = new Thread(ultrasonic);
			UltrasonicThread.start();

			// Start LightSensorPoller
			Thread lightThread = new Thread(light);
			lightThread.start();

			// spawn a new Thread to avoid UltrasonicLocalizer.run() from blocking
			(new Thread() {
				public void run() {
					UltrasonicLocalizer.run(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer);
				}
			}).start();

			while(true) {										 // run the Light Localizer
				if (UltrasonicLocalizer.finished = true) {		 // check if Ultrasonic Localizer is finished
					(new Thread() {	   // spawn a new Thread to avoid LightLocalizer.run() from blocking
						public void run() {
							LightLocalizer.run(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer);
						}
					}).start();
					break;
				}
			}
			
			//TODO: add search procedure
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
