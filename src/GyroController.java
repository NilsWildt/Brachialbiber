package de.nilswildt.de;

import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;


public class GyroController {
	
	EV3GyroSensor sensor;
	NXTMotor motor;
	
	//Feste Werte des Controllers
	private final double maxAngle = 20.0; //maximale Winkelgeschwindigkeit, bis zu der wir regulieren
	private final double maxMotorSpeed = 50.0; //Power des Motors bei maximaler Winkeländerung //TODO Werte anpassen
	private final double Kp = maxMotorSpeed/maxAngle; //TODO ergibt sich rechnerisch, Verwendung in setMotion()!
	private final double Ki = 0.0;
	private final double Kd = 0.0;
	
	double angle;
	
	/**
	 * Constructor
	 * 
	 * @author Markus Schmidgall
	 */
	public GyroController(EV3GyroSensor gyroSensor, NXTMotor nxtMotor){
		sensor = gyroSensor;
		motor = nxtMotor;
	}
	
	/**
	 * Initialsiert die Startwerte des Gyrosensors (Wert wird auf 0 gesetzt (hopefully XD))
	 */
	public void initalize(){
		sensor.reset(); //Das is iwie n bissl unsave, es setzt den Gyro in Mode 4, aber ich hab kein Plan was der macht.
		//TODO rausfinden, was das tut
	}
	
	/**
	 * @return Momentaner Winkel des Gyros
	 */
	public float getAngle(){
		float[] angle = new float[1];
		sensor.getAngleMode().fetchSample(angle, 0); //hier wird der Winkel ausgelesen
		return angle[0];
	}
	/**
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er möglichst gerade bleibt
	 * 
	 * @param angle Momentaner Winkel des Gyrosensors
	 */
	public void setMotion(float currAngle){
		int newPower;
		
		motor.flt(); //weiß ned, ob man das braucht
		angle = (double) currAngle;
		
		if(angle < 0.0){ //TODO je nachdem wierum der Sensor angebracht ist, müssen das noch ändern (>)
			motor.backward();
			angle = -angle;
		} else{
			motor.forward();
		}
		
		angle = Math.max(angle, maxAngle); //So das wir nicht über unseren maximalen Steuerungsbereich hinauskommen
		newPower = (int)(Kp*angle);
		motor.setPower(newPower);
	}
}
