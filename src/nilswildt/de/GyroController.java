package nilswildt.de;

import java.io.IOException;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroController {
	EV3GyroSensor sensor;
	EV3TouchSensor touch;
	NXTMotor motor;

	// Feste Werte des Controllers
	private double Kp = 3.9; // ergibt sich empirisch, Verwendung in setMotion()!
	private final double BLOCK_DOWN = 0.9; // -0.9
	private double newPower = 0; // from setMotion
	private double angleVelocity = 0.0;
	private double sumPower = 0.0;
	private float[] isTouched = new float[1];
	int cnt = 0;
	WriteFile write;
	private SampleProvider provider;

	/**
	 * Constructor
	 * 
	 * @author Markus Schmidgall
	 * @throws IOException
	 */
	public GyroController(EV3GyroSensor gyroSensor, EV3TouchSensor touchSensor, NXTMotor nxtMotor) {
		sensor = gyroSensor;
		touch = touchSensor;
		motor = nxtMotor;
		provider = sensor.getRateMode();
		write = new WriteFile();
		setGyroKP(3.9); //Wird ab blau auf 4.0 gesetzt, siehe main
	}

	/**
	 * Initialsiert die Startwerte des Gyrosensors (Wert wird auf 0 gesetzt (hopefully XD))
	 */
	public void initalizeGyroController() {
		Brachialbiber.printer("Press touch to continue");
		Sound.beep();
		do{
			touch.fetchSample(isTouched, 0);
		}while(isTouched[0] != 1f);
		LCD.clear();
		Brachialbiber.printer("Gyrosensor wird initialisiert.");
		Delay.msDelay(3000);
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
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er mÃƒÂ¶glichst gerade bleibt
	 * 
	 * @param angle
	 *            Momentane Winkelgeschwindigkeit des Gyrosensors
	 */
	public void setMotion() {
		angleVelocity = (double) getAngleVelocity(); // Über wie viel gemittelt wird;
		//write.writeToFile(angleVelocity); // Log angleVelocity
		// Calculate Power
		newPower = (Kp * angleVelocity);
		newPower = (-1) * Math.signum(newPower)* Math.min(Math.abs(newPower), 100);

		if (newPower * sumPower < 0) {
			cnt++;
			if (cnt >= 3) {
				sumPower = 0;
				cnt = 0;
			}
		} else {
			cnt = 0;
		}
	//	if (newPower > 0) {
		//	newPower *= BLOCK_DOWN;
	//	}
		if (Math.abs(newPower) >= 30) {
			if (newPower < 0.0) {
				motor.backward();
				sumPower *= BLOCK_DOWN;
			} else {
				motor.forward();
			}
			motor.setPower((int) Math.round(Math.abs(newPower)));
		} else if (Math.abs(newPower) > 2) { // zwischen null und |30|
			sumPower += newPower;
			if (Math.abs(sumPower) >= 30) {
				if (sumPower < 0.0) {
					motor.backward();
					sumPower *= BLOCK_DOWN;
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
	
	public void setGyroKP(double kp){
		this.Kp=kp;
	}
}
