/**
 * 
 */
package nilswildt.de;

import org.jfree.data.statistics.MeanAndStandardDeviation;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

/**
 * @author nildt
 *
 *         Klasse zum Regeln der Linienfolgefunktionalität
 */
public class LineController {

	private EV3ColorSensor sensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	// Constants
	private double KP;
	private double KI;
	private double KD;

	private final static int acceleration = 6000; // Default = 6000

	private float[] sample;
	private SampleProvider rgbMode;
	private float speed;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double currentValue; // Immer der aktuelle Helligkeitswert
	// TODO BlueValue?? Je nach Implementation
	private double error;
	private double turn;

	/*
	 * Constructor
	 */
	public LineController(EV3ColorSensor sensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.sensor = sensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		rgbMode = this.sensor.getRGBMode();
		sample = new float[rgbMode.sampleSize() + 1];
		speed = 150; //Math.max(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed()); // TODO Austesten! /Zusammen mit													// Acceleration
		KP = speed / 8;
		// Generate sampleArray:
		sample = new float[4]; // rgb,intensity at [3]
	}

	public void initializeLineController() {
		/* Read the High Value */
		Brachialbiber.printer("Press Enter to calibrate high");
		Delay.msDelay(1000);
		LCD.clear();
		while (Button.ENTER.isUp()) {
			this.brightValue = fetchCurrentValues()[3];
			System.out.println(brightValue);
		}
		// Last value is set now.
		this.brightValue = fetchCurrentValues()[3];
		LCD.clear();
		Delay.msDelay(500);
		/* Read the Low Value */
		Brachialbiber.printer("Press Button to calibrate low");
		Delay.msDelay(1000);
		LCD.clear();
		while (Button.ENTER.isUp()) {
			this.darkValue = fetchCurrentValues()[3];
			System.out.println(darkValue);
		}
		LCD.clear();
		// Last value is set now.
		this.darkValue = fetchCurrentValues()[3];

		Brachialbiber.printer("Bright: " + brightValue + "Dark: " + darkValue);
		Delay.msDelay(1000);
		LCD.clear();
		midValue = (brightValue + darkValue) / 2; // Mittelwert
		LCD.clear();
		Brachialbiber
				.printer("Justiere jetzt den Biber, so dass folgender Wert immer 0 ist! Drücke enter, wenn fertig.");
		Delay.msDelay(1000);
		LCD.clear();
		while (Button.ENTER.isUp()) {
			System.out.println(fetchCurrentValues()[3] - midValue);
		}
	}

	/*
	 * startet die beiden Motoren mit den Anfangswerten.
	 */
	public void startMotor() {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		
		
		leftMotor.forward();
		rightMotor.forward();

	}

	/**
	 * Only P-controlled TODO I D part...
	 */
	public void drive() {
		currentValue = fetchCurrentValues()[3];
		error = midValue - currentValue;
		turn = KP * error;
		leftMotor.setSpeed((int) Math.ceil(speed - turn));
		rightMotor.setSpeed((int) Math.ceil(speed + turn));
	}

	/**
	 * Gets the current Light values.
	 */
	private float[] fetchCurrentValues() {
		// SampleProvider average = new MeanFilter(rgbMode, 5);
		// average.fetchSample(sample, 0);
		rgbMode.fetchSample(sample, 0);
		for (int i = 0; i < sample.length; i++) {
			sample[i] = sample[i] * 100;
		}
		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(sample[2], 2))); // Intensity
		return sample;
	}

}
