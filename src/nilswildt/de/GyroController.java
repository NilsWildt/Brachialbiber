package nilswildt.de;

import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;


public class GyroController {
	
	EV3GyroSensor sensor;
	NXTMotor motor;
	
	//Feste Werte des Controllers
	private final double MAX_ANGLE_VELOCITY = 20.0; //maximale Winkelgeschwindigkeit, bis zu der wir regulieren
	private final double MAX_MOTOR_SPEED_P = 105.0; //Power des Motors bei maximaler WinkelÃ¤nderung //TODO Werte anpassen
	private final double Kp = MAX_MOTOR_SPEED_P / MAX_ANGLE_VELOCITY; //ergibt sich rechnerisch, Verwendung in setMotion()!
	
	private int newPower = 0; // from setMotion
	
	private double angleVelocity = 0.0;
	
	
	
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
		System.out.println("Init Sensor");
		sensor.reset(); //Das is iwie n bissl unsave, es setzt den Gyro in Mode 4, aber ich hab kein Plan, was der macht.
		//TODO rausfinden, was das tut
		System.out.println("Sensor initialized.");
	}
	
	/**
	 * @return Momentane Winkelgeschwindigkeit des Gyros
	 * TODO Filter? Median? Mittelwert?
	 */
	public float getAngleVelocity(){
		float[] angleVelocity = new float[1];
		sensor.getRateMode().fetchSample(angleVelocity, 0);
		return  angleVelocity[0];
	}
	/**
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er möglichst gerade bleibt
	 * 
	 * @param angle Momentane Winkelgeschwindigkeit des Gyrosensors
	 */
	public void setMotion(){		
		//motor.flt(); //weiß ned, ob man das braucht
		//motor.setPower(newPower/6); //TODO
		
		angleVelocity = (double) getAngleVelocity();
		System.out.println(Math.signum(angleVelocity));
		if(angleVelocity < 0.0){ //TODO je nachdem wieherum der Sensor angebracht ist, müssen das noch Ã¤ndern (>)
			motor.backward();
			angleVelocity = -0.6*angleVelocity;
		} else{
			motor.forward();
		}
		
		//Calculate Power
		newPower = (int)(Kp*angleVelocity);
		newPower = Math.min(newPower, 100);
		motor.setPower(newPower);
	}
}
