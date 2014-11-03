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
	private double Kp = 1.5; // ergibt sich empirisch, Verwendung in setMotion()!
	private final double BLOCK_DOWN = -0.9;
	private int newPower = 0; // from setMotion
	private double angleVelocity = 0.0;
	
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
	
	public void setKP(double value){
		this.Kp = value;
	}

	/**
	 * Initialsiert die Startwerte des Gyrosensors (Wert wird auf 0 gesetzt (hopefully XD))
	 */
	public void initalizeGyroController() {
		Brachialbiber.printer("Gyrosensor wird initialisiert.");
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
	public float getAngleVelocity(int times) {
		float sum = 0f;
		float[] angleVelocity = new float[1];
		for(int i=0; i<times; i++){
			provider.fetchSample(angleVelocity, 0);
			sum += angleVelocity[0];
		}
		provider.fetchSample(angleVelocity, 0);
		return sum;
	}

	/**
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er mÃ¶glichst gerade bleibt
	 * 
	 * @param angle
	 *            Momentane Winkelgeschwindigkeit des Gyrosensors
	 */
	public void setMotion() {
		angleVelocity = (double) getAngleVelocity(3);
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
		System.out.println(newPower);
	}
}
