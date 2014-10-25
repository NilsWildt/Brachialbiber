package de.nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Brachialbiber {
	private EV3ColorSensor colSensor;
	private float[] colorSensorSample;
	private SampleProvider redMode;
	// private SampleProvider rgbMode;
	private float brightValue;
	private float darkValue;

	/**
	 * Contructor
	 * 
	 * @param colSenPort
	 *            Port of the color sensor
	 */
	public Brachialbiber(Port colSenPort) {
		colSensor = new EV3ColorSensor(colSenPort);
		redMode = colSensor.getRedMode();
		// rgbMode = colSensor.getRGBMode();
		colorSensorSample = new float[redMode.sampleSize()];
	}

	/**
	 * Calibrate the ColorSensor
	 */
	public void initialize() {
		/* Read the High Value */
		LCD.drawString("Calibrate bright", 0, 0);
		Button.waitForAnyPress();
		this.brightValue = fetchCurrentValue(); // TODO Mitteln!
		LCD.clear();
		/* Read the Low Value */
		LCD.drawString("Calibrate dark", 0, 0);
		Button.waitForAnyPress();
		this.darkValue = fetchCurrentValue();
		LCD.clear();
		System.out.println("High: " + Float.toString(brightValue) + '\n'
				+ "Low: " + Float.toString(darkValue));
		Delay.msDelay(2000);
	}

	protected void view() {
		LCD.drawString("Zeige RedMode Samples", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		// First: redmode:
		while (!Button.ESCAPE.isDown()) {
			System.out.println(fetchCurrentValue());
			Delay.msDelay(10);
		}

	}

	/**
	 * Fetches the current reflected light value.
	 * 
	 * @return reflected light value between 0 and 100
	 */
	private float fetchCurrentValue() {
		redMode.fetchSample(colorSensorSample, 0);
		return colorSensorSample[0] * 100; // In percent!
	}

	public void close() {
		LCD.clear();
		System.out.println("Finished!");
		Delay.msDelay(1000);
		Sound.beep();
		colSensor.close();
		System.exit(1);
	}

}