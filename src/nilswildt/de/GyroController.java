package nilswildt.de;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroController {

	EV3GyroSensor sensor;
	NXTMotor motor;

	// Feste Werte des Controllers
	private double Kp = 3.2; // ergibt sich empirisch, Verwendung in setMotion()!
	private final double BLOCK_DOWN = -1; // -0.9
	private int newPower = 0; // from setMotion
	private double angleVelocity = 0.0;
	private double sumPower = 0.0;

	private SampleProvider provider;

	/**
	 * Constructor
	 * 
	 * @author Markus Schmidgall
	 */
	public GyroController(EV3GyroSensor gyroSensor, NXTMotor nxtMotor) {
		sensor = gyroSensor;
		motor = nxtMotor;
		provider = sensor.getRateMode();
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
		// Calculate Power
		newPower = (int) (Kp * angleVelocity);
		newPower = Math.min(newPower, 100);

		sumPower += newPower;
		
		
		if (sumPower >= 30 || sumPower <= -30) {
			if (sumPower < 0.0) {
				motor.backward();
				sumPower = BLOCK_DOWN * sumPower;
			} else {
				motor.forward();
			}

			motor.setPower((int) sumPower);
			sumPower = 0.0;
		} else {
			motor.setPower(0);
			//motor.flt(); //Klappt ganz gut...
		}

	}
}
