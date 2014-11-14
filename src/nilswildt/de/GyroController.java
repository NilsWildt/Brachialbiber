package nilswildt.de;

import java.io.IOException;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroController {

	EV3GyroSensor sensor;
	NXTMotor motor;

	// Feste Werte des Controllers
	private double Kp = 4.0; // ergibt sich empirisch, Verwendung in setMotion()!
	private final double BLOCK_DOWN = 0.9; // -0.9
	private double newPower = 0; // from setMotion
	private double angleVelocity = 0.0;
	private double sumPower = 0.0;
	int cnt = 0;
	WriteFile write;
	private SampleProvider provider;

	/**
	 * Constructor
	 * 
	 * @author Markus Schmidgall
	 * @throws IOException
	 */
	public GyroController(EV3GyroSensor gyroSensor, NXTMotor nxtMotor) {
		sensor = gyroSensor;
		motor = nxtMotor;
		provider = sensor.getRateMode();
		write = new WriteFile();
	}

	/**
	 * Initialsiert die Startwerte des Gyrosensors (Wert wird auf 0 gesetzt (hopefully XD))
	 */
	public void initalizeGyroController() {
		Brachialbiber.printer("Gyrosensor wird initialisiert.");
		Delay.msDelay(1000);
		LCD.clear();
		sensor.reset();
		Delay.msDelay(1000);
		Brachialbiber.printer("Gyrosensor fertig initialisiert");
	}

	/**
	 * @return Momentane Winkelgeschwindigkeit des Gyros TODO Filter? Median? Mittelwert?
	 */
	public float getAngleVelocity() {
		float[] angleVelocity = new float[1];
		provider.fetchSample(angleVelocity, 0);
		return angleVelocity[0];

	}

	/**
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er mÃ¶glichst gerade bleibt
	 * 
	 * @param angle
	 *            Momentane Winkelgeschwindigkeit des Gyrosensors
	 */
	public void setMotion() {
		angleVelocity = (double) getAngleVelocity(); // �ber wie viel gemittelt wird;
		write.writeToFile(angleVelocity); // Log angleVelocity
		// Calculate Power
		newPower = (Kp * angleVelocity);
		newPower = (-1) * Math.signum(newPower)
				* Math.min(Math.abs(newPower), 100); // Damit auch bei negativen

		if (newPower * sumPower < 0) {
			cnt++;
			if (cnt >= 3) {
				sumPower = 0;
				cnt = 0;
			}
		} else {
			cnt = 0;
		}
		if (newPower > 0) {
			newPower *= BLOCK_DOWN;
		}
		if (Math.abs(newPower) >= 30) {
			if (newPower < 0.0) {
				motor.backward();
			} else {
				motor.forward();
			}
			motor.setPower((int) Math.round(Math.abs(newPower)));
		} else if (Math.abs(newPower) > 2) { // zwischen null und |30|
			sumPower += newPower;
			if (Math.abs(sumPower) >= 30) {
				if (sumPower < 0.0) {
					motor.backward();
				} else {
					motor.forward();
				}
				motor.setPower((int) Math.round(Math.abs(sumPower)));
				sumPower = 0.0;
			}
		} else {
			motor.setPower(0);
		}
	}

	public void interruptMotion() {
		this.motor.setPower(0);
	}
}
