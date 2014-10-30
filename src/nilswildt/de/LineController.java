/**
 * 
 */
package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;


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
	private double KP = 0.0; // Nur zum Testen, siehe drive()

	private final double KI = 0.0;
	private final double KD = 0.0;

	private float[] sample;
	private SampleProvider rgbMode;
	private float speed;
	private int acceleration = 0;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double currentValue; // Immer der aktuelle Helligkeitswert
	// TODO BlueValue?? Je nach Iplementation
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
			
		rgbMode = sensor.getRedMode();
		
		sample = new float [rgbMode.sampleSize()+1];
		KP	= speed / 8;

		// Generate sampleArray:
		sample = new float[4]; // rgb,intensity at [3]
	}

	public void initializeLineController() {
		/* Read the High Value */
		Brachialbiber.printer("PressButton to calibrate high");
		while (Button.ENTER.isUp()) {
			this.brightValue = fetchCurrentValues()[3];
			System.out.println(brightValue);
		}
		LCD.clear();
		// Last value is set now.
		this.brightValue = fetchCurrentValues()[3];

		/* Read the Low Value */
		Brachialbiber.printer("PressButton to calibrate low");
		while (Button.ENTER.isUp()) {
			this.darkValue = fetchCurrentValues()[3];
			System.out.println(darkValue);
		}
		LCD.clear();
		// Last value is set now.
		this.darkValue = fetchCurrentValues()[3];

		Brachialbiber.printer("Bright: " + brightValue + "Dark: " + darkValue);
		midValue = (brightValue + darkValue) / 2; // Mittelwert
		LCD.clear();
		Brachialbiber
				.printer("Justiere jetzt den Biber, so dass folgender Wert immer 0 ist! Drücke enter, wenn fertig.");
		while (Button.ENTER.isUp()) {
			System.out.println(fetchCurrentValues()[3] - midValue);
		}
	}

	/*
	 * startet die beiden Motoren mit den Anfangswerten.
	 */
	public void startMotor() {
		speed = Math.min(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed());

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);

		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * Only P-controlled
	 * TODO I D part...
	 */
	public void drive() {
		currentValue = fetchCurrentValues()[3];
		error = midValue * 10 - currentValue * 10;
		turn = KP * error;
		leftMotor.setSpeed((int) Math.ceil(speed - turn));
		rightMotor.setSpeed((int) Math.ceil(speed + turn));
	}

	/**
	 * Gets the current Light values.
	 */
	private float[] fetchCurrentValues() {
		rgbMode.fetchSample(sample, 0);
		System.out.println(sample);
		// TODO calculate intensity into [3]
		return sample;
	}

}
