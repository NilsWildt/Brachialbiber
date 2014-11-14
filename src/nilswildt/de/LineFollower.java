package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

public class LineFollower {

	private EV3ColorSensor sensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private final static int acceleration = 1000;
	
	private float[] sample;
	private SampleProvider rgbMode;
	private SampleProvider average;
	private float speed;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double currentValue; // Immer der aktuelle Helligkeitswert
	private double error;
	private double turn;
	
	//Kc = 12
	private final double Pc = 0.792; //Periodendauer
	private final double dT = 0.02312; //Durchlauf 
	
	private double KP;
	private double KI;
	private double KD;
	
	private double speedLeft;
	private double speedRight;
	
	private double integral;
	private double derivative;
	private double lastError;
	
	
	public LineFollower(EV3ColorSensor sensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.sensor = sensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		rgbMode = this.sensor.getRGBMode();
		speed = Math.max(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed())/4.0f; // Acceleration
		KP = 11f;//8.4f;//speed / 8.0;
		KI = 0.0f;//0.05f;//speed * 0.0;
		KD = 0.0f;//35.6f;//speed * 0.0;
		sample = new float[4]; // rgb,intensity at [3]
		integral = 0.0;
		derivative = 0.0;
		lastError = 0.0;
	}
	
	public void initFollower() {
		double value;
		LCD.clear();
		LCD.drawString("Calibrate Brigth-Value: (Press ENTER-Button)",0,0);
		while(Button.ENTER.isUp()){
			brightValue = (double) fetchAverageSample()[3];
		}
		Delay.msDelay(500);
		LCD.clear();
		LCD.drawString("Calibrate Dark-Value: (Press ENTER-Button)",0,0);
		while(Button.ENTER.isUp()){
			darkValue = (double) fetchAverageSample()[3];
		}
		Delay.msDelay(500);
		midValue = (brightValue + darkValue) / 2.0;
		LCD.clear();
		Brachialbiber.printer("Justiere auf 0");
		Delay.msDelay(1000);
		LCD.clear();
		while(Button.ENTER.isUp()) {
			value = Math.round((fetchAverageSample()[3] - midValue)*100.0)/100.0;
			System.out.println(value);
		}
		
		//Werte an Strecke anpassen
		KP = 10*(25)/(brightValue - darkValue);
		// Uncomment to disable I and D Term!
		KI = 2.0*KP*dT/Pc;
		KD = KP*Pc/(8.0*dT);
		LCD.clear();
	}
	
	public void initMotor(){
			leftMotor.setAcceleration(acceleration);
			rightMotor.setAcceleration(acceleration);
			
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			
			leftMotor.forward();
			rightMotor.forward();
	}
	
	public void drive() {
		currentValue = fetchAverageSample()[3];
		
		//P-Part
		error = midValue - currentValue;
		
		//I-Part
		if(Math.signum(error) != Math.signum(integral)) integral = 0.0;
		integral = integral + error; //(2.0/3.0)*
		
		//D-Part
		derivative = error - lastError;
		
		turn = KP * error + KI * integral + KD * derivative;
		speedLeft = Math.round(Math.max(Math.min(speed - turn, 2*speed), 1));
		speedRight = Math.round(Math.max(Math.min(speed + turn, 2*speed), 1));
		
		
		leftMotor.setSpeed((int) speedLeft);
		rightMotor.setSpeed((int) speedRight);
		
		lastError = error;
		LCD.clear();

	}
	
	public float[] fetchSample(){
		rgbMode.fetchSample(sample, 0);
		for (int i=0; i<sample.length-1; i++) {
			sample[i] = sample[i] * 100f;
		}
		
		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2*sample[2], 2)));
		return sample;
	}	
	
	public float[] fetchAverageSample(){
		average = new MeanFilter(rgbMode, 5);
		average.fetchSample(sample, 0);
		for (int i=0; i<sample.length-1; i++) {
			sample[i] = sample[i] * 100f;
		}
		
		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2*sample[2], 2))); // TODO ÜBERARBEITEN
		return sample;
	}
	
	public boolean isBlue(){
		sample = fetchAverageSample();
		if(sample[0] < sample[2]*1.25f) return true;
		return false;
	}
	
	public void stopMotor(){
		leftMotor.flt();
		rightMotor.flt();
	}
	
}
