package nilswildt.de;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.utility.Delay;

public class GyroController {

	EV3GyroSensor sensor;
	NXTMotor motor;

	// Feste Werte des Controllers
	private final double Kp = 5.25; // ergibt sich empirisch, Verwendung in setMotion()!
	private final double BLOCK_DOWN = -0.9;
	private int newPower = 0; // from setMotion
	private double angleVelocity = 0.0;

	/**
	 * Constructor
	 * 
	 * @author Markus Schmidgall
	 */
	public GyroController(EV3GyroSensor gyroSensor, NXTMotor nxtMotor) {
		sensor = gyroSensor;
		motor = nxtMotor;
	}

	/**
	 * Initialsiert die Startwerte des Gyrosensors (Wert wird auf 0 gesetzt (hopefully XD))
	 */
	public void initalizeGyroController() {
		Brachialbiber.printer("Gyrosenesor wird initialisiert.");
		Delay.msDelay(500);
		LCD.clear();
		sensor.reset(); // Das is iwie n bissl unsave, es setzt den Gyro in Mode 4, aber ich hab kein Plan, was der
						// macht.
		// TODO rausfinden, was das tut
		Brachialbiber.printer("Gyrosensor fertig initialisiert");
	}

	/**
	 * @return Momentane Winkelgeschwindigkeit des Gyros TODO Filter? Median? Mittelwert?
	 */
	public float getAngleVelocity() {
		float[] angleVelocity = new float[1];
		sensor.getRateMode().fetchSample(angleVelocity, 0);
		return angleVelocity[0];
	}

	/**
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er möglichst gerade bleibt
	 * 
	 * @param angle
	 *            Momentane Winkelgeschwindigkeit des Gyrosensors
	 */
	public void setMotion() {
		angleVelocity = (double) getAngleVelocity();
		//System.out.println(Math.signum(angleVelocity));
		if (angleVelocity < 0.0) {
			motor.backward();
			angleVelocity = BLOCK_DOWN * angleVelocity;
		} else {
			motor.forward();
		}

		// Calculate Power
		newPower = (int) (Kp * angleVelocity);
		newPower = Math.min(newPower, 100);
		motor.setPower(newPower);
	}
}
