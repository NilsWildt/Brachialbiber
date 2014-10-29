package nilswildt.de;

import lejos.hardware.motor.NXTMotor;
import lejos.hardware.sensor.EV3GyroSensor;


public class GyroController {
	
	EV3GyroSensor sensor;
	NXTMotor motor;
	
	//Feste Werte des Controllers
	private final double MAX_ANGLE_VELOCITY = 60.0; //maximale Winkelgeschwindigkeit, bis zu der wir regulieren
	private final double MAX_MOTOR_SPEED_P = 50.0; //Power des Motors bei maximaler Winkeländerung //TODO Werte anpassen
	private final double MAX_MOTOR_SPEED_I = 10.0; //Maximale Power, die zusätzlich noch vom Integralteil kommt
	private final double MAX_MOTOR_SPEED_D = 10.0; //Maximale Power, die zusätzlich noch vom Derivativeteil kommt
	private final double I_FACTOR = 2.0/3.0; //Für ein vergessendes Integral (gibt den Bruchteil des Integrals an den wir behalten)
	private final double Kp = MAX_MOTOR_SPEED_P / MAX_ANGLE_VELOCITY; //ergibt sich rechnerisch, Verwendung in setMotion()!
	private final double Ki = (1 - I_FACTOR) * MAX_MOTOR_SPEED_I / MAX_ANGLE_VELOCITY; //sollten wir I_FACTOR nicht übernehmen, müssen wir diese Formel ändern!
	private final double Kd = MAX_MOTOR_SPEED_D / (2 * MAX_ANGLE_VELOCITY);
	
	double angleVelocity = 0;
	double integralAngle = 0;
	double derivativeAngle = 0.0;
	double lastAngleVelocity = 0.0;
	
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
		sensor.reset(); //Das is iwie n bissl unsave, es setzt den Gyro in Mode 4, aber ich hab kein Plan, was der macht.
		//TODO rausfinden, was das tut
	}
	
	/**
	 * @return Momentane Winkelgeschwindigkeit des Gyros
	 */
	public float getAngleVelocity(){
		float[] angleVelocity = new float[1];
		sensor.getAngleMode().fetchSample(angleVelocity, 0); //hier wird der Winkel ausgelesen
		return angleVelocity[0];
	}
	/**
	 * Diese Methode korrigiert den Winkel des Eihalters, so dass er möglichst gerade bleibt
	 * 
	 * @param angle Momentane Winkelgeschwindigkeit des Gyrosensors
	 */
	public void setMotion(float currAngleVelocity){
		int newPower;
		
		motor.flt(); //weiß ned, ob man das braucht
		angleVelocity = (double) currAngleVelocity;
		
		if(angleVelocity < 0.0){ //TODO je nachdem wierum der Sensor angebracht ist, müssen das noch ändern (>)
			motor.backward();
			angleVelocity = -angleVelocity;
		} else{
			motor.forward();
		}
		
		//The "P"-Part
		angleVelocity = Math.max(angleVelocity, MAX_ANGLE_VELOCITY); //So, dass wir nicht über unseren maximalen Steuerungsbereich hinauskommen
		
		//The "I"-Part
		if(Math.signum(angleVelocity) != Math.signum(integralAngle)) integralAngle = 0; //weiß ned, ob das nötig ist
		integralAngle = I_FACTOR*integralAngle + angleVelocity; //Vergessendes Integral
		
		//The "D"-Part
		derivativeAngle = angleVelocity - lastAngleVelocity;
		
		//Calculate Power
		newPower = (int)(Kp*angleVelocity + Ki*integralAngle + Kd*derivativeAngle);
		motor.setPower(newPower);
		
		lastAngleVelocity = angleVelocity; //Merkt sich den letzen Error für den Derivative-Part
	}
}
