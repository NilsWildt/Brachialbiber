package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

	private EV3ColorSensor sensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private final static int acceleration = 1000;
	
	private float[] sample;
	private float[][] sampleSample;
	private SampleProvider rgbMode;
	private float speed;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double currentValue; // Immer der aktuelle Helligkeitswert
	private double error;
	private double turn;
	
	private final double KP;
	private final double KI;
	private final double KD;
	
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
		speed = Math.max(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed())/10f; // Acceleration
		KP = 10.0f;//speed / 8.0;
		KI = 0.001f;//speed * 0.0;
		KD = 10.0f;//speed * 0.0;
		sample = new float[4]; // rgb,intensity at [3]
		sampleSample = new float[3][4];
		integral = 0.0;
		derivative = 0.0;
		lastError = 0.0;
	}
	
	public void initFollower() {
		double value;
		LCD.clear();
		LCD.drawString("Calibrate Brigth-Value: (Press ENTER-Button)",0,0);
		while(Button.ENTER.isUp()){
			brightValue = (double) fetchMiddleSample()[3];
		}
		Delay.msDelay(200);
		LCD.clear();
		LCD.drawString("Calibrate Dark-Value: (Press ENTER-Button)",0,0);
		while(Button.ENTER.isUp()){
			darkValue = (double) fetchMiddleSample()[3];
		}
		Delay.msDelay(200);
		midValue = (brightValue + darkValue) / 2.0;
		LCD.clear();
		Brachialbiber.printer("Justiere auf 0");
		Delay.msDelay(1000);
		LCD.clear();
		while(Button.ENTER.isUp()) {
			value = Math.round((fetchMiddleSample()[3] - midValue)*100.0)/100.0;
			System.out.println(value);
		}
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
		currentValue = fetchMiddleSample()[3];
		
		//P-Part
		error = midValue - currentValue;
		
		//I-Part
		if(Math.signum(error) != Math.signum(integral)) integral = 0.0;
		integral = integral + error; //(2.0/3.0)*
		
		//D-Part
		derivative = error - lastError;
		
		turn = KP * error + KI * integral + KD * derivative;
		speedLeft = Math.max(Math.min(speed - turn, 2*speed), 1);
		speedRight = Math.max(Math.min(speed + turn, 2*speed), 1);
		
		leftMotor.setSpeed((int) Math.round(speedLeft));
		rightMotor.setSpeed((int) Math.round(speedRight));
		
		lastError = error;
		LCD.clear();
		System.out.println(leftMotor.getSpeed() + "  " + rightMotor.getSpeed());
	}
	
	public void testDrive(){
		
	}
	
	public float[] fetchSample(){
		average = new MeanFilter(rgbMode, 5);
		average.fetchSample(sample, 0);
		for (int i=0; i<sample.length-1; i++) {
			sample[i] = sample[i] * 100f;
		}
		
		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2*sample[2], 2))); // TODO ÜBERARBEITEN
		return sample;
	}	
	
	public float[] fetchMiddleSample(){
		for (int i=0; i<sample.length-1; i++) {
			sample[i] = sample[i] * 100f;
		}
		
		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2*sample[2], 2))); // TODO ÜBERARBEITEN
		return sample;
		/*
		for(int j=0; j<sampleSample.length; j++){
			rgbMode.fetchSample(sample, 0);
			for (int i=0; i<sample.length-1; i++) {
				sample[i] = sample[i] * 100f;
				sampleSample[j][i] = sample[i];
			} 
		}
		
		for(int k=0; k<sampleSample.length; k++){
			sample[k] = (sampleSample[k][1] + sampleSample[k][2] + sampleSample[k][3])/3f;
		}
		
		sample[3] = (float) Math.sqrt((Math.pow(sample[0], 2)
				+ Math.pow(sample[1], 2) + Math.pow(2*sample[2], 2))); // TODO ÜBERARBEITEN
		return sample;*/
	}
	
}