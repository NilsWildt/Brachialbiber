package nilswildt.de;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

public class LineFollower {

	private EV3ColorSensor sensor;
	private EV3TouchSensor touchSensor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private final static int acceleration = 1000;
	
	private float[] sample;
	private float[] isTouched;
	private SampleProvider rgbMode;
	private SampleProvider average;
	private float speed;
	private double brightValue; // Höchster Helligkeitswert
	private double darkValue; // Höchster Schwarzwert (Linie)
	private double midValue; // Mitte zwischen Hell und dunkel
	private double range;
	private double currentValue; // Immer der aktuelle Helligkeitswert
	private double error;
	private double turn;
	
	private final double KC = 5.0;
	private final double PC = 0.66275; //Periodendauer
	private final double dT = 0.0204; //Durchlauf 
	
	private double KP;
	private double KI;
	private double KD;
	private double maxError;
	private double errorChange;
	private double errorShift;
	
	private double speedLeft;
	private double speedRight;
	
	private double integral;
	private double derivative;
	private double lastError;
	
	double KK = 1.1;
	
	public LineFollower(EV3ColorSensor sensor, EV3TouchSensor touchSensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.sensor = sensor;
		this.touchSensor = touchSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		rgbMode = this.sensor.getRGBMode();
		sensor.setFloodlight(true);
		speed = 350.0f;
		leftMotor.setAcceleration(100);
		rightMotor.setAcceleration(100);
		KP = 5.1;//5.0;
		KI = 0.011;//0.02;
		KD = 21.0;//8.0;
		maxError = 1.2;
		errorChange = -0.03 * 0.1;	
		errorShift = 1.0;		
		sample = new float[4]; // rgb,intensity at [3]
		isTouched = new float[1];
		integral = 0.0;
		derivative = 0.0;
		lastError = 0.0;
	}
	
	public void initFollower() {
		double value;
		LCD.clear();
		
		LCD.drawString("Calibrate Brigth-Value: (Press ENTER-Button)",0,0);
		do{
			brightValue = (double) fetchAverageSample();
			touchSensor.fetchSample(isTouched, 0);
		}while(isTouched[0] != 1f);
		Delay.msDelay(500);
		LCD.clear();
		LCD.drawString("Calibrate Dark-Value: (Press ENTER-Button)",0,0);
		do{
			darkValue = (double) fetchAverageSample();
			touchSensor.fetchSample(isTouched, 0);
		}while(isTouched[0] != 1f);
		Delay.msDelay(500);
		
		midValue = (brightValue + darkValue) / 2.0;
		LCD.clear();
		Brachialbiber.printer("Justiere auf 0");
		Delay.msDelay(1000);
		LCD.clear();
		do{
			touchSensor.fetchSample(isTouched, 0);
			value = fetchAverageSample() - midValue;//Math.round((fetchAverageSample() - midValue)*100.0)/100.0;
			System.out.println(Math.round(value*100.0)/100.0);
		}while(isTouched[0] != 1f);
		
		range = Math.abs(brightValue - darkValue)/2.0;
		
		//Werte an Strecke anpassen
		//KP = 10*(25)/(brightValue - darkValue);
		//Uncomment to disable I and D Term!
		//KI = 2.0*KP*dT/PC;
		//KD = KP*PC/(8.0*dT);
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
		currentValue = fetchAverageSample();
		
		//P-Part
		error = midValue - currentValue;
		//KP = Math.max(1.0, Math.abs(error)/range*10.0);
		//error = Math.signum(error)*maxError*range/(1+Math.pow(Math.E, errorChange*range*(Math.abs(error) - errorShift*range/2.0)));
		
		//I-Part
		if(Math.signum(error) != Math.signum(integral)) integral = 0.0;
		integral = 0.8*integral + error; //(2.0/3.0)*
		
		//D-Part
		//KD = 20.0/Math.pow(Math.E, 0.1*Math.abs(error));
		derivative = error - lastError;
		
		turn = KP * error + KI * integral + KD * derivative;
		speedLeft = Math.round(Math.max(Math.min(speed - turn, 2*speed), 1));
		speedRight = Math.round(Math.max(Math.min(speed + turn, 2*speed), 1));
		
		
		leftMotor.setSpeed((int) speedLeft);
		rightMotor.setSpeed((int) speedRight);
		
		lastError = error;
		LCD.clear();
//		Delay.msDelay(5);
	}
	
	public float fetchSample(){
		rgbMode.fetchSample(sample, 0);
		for(int i=0; i<3; i++)
			sample[i] *= 100f;
		sample[3] = (float) Math.sqrt(Math.pow(sample[0], 2) + Math.pow(sample[1], 2) + Math.pow(2.0*sample[2], 2));
		return sample[3];
	}	
	
	public float fetchAverageSample(){
		average = new MeanFilter(rgbMode, 5);
		average.fetchSample(sample, 0);
		for(int i=0; i<3; i++)
			sample[i] *= 100f;
		sample[3] = (float) Math.sqrt(Math.pow(sample[0], 2) + Math.pow(sample[1], 2) + Math.pow(2.0*sample[2], 2));
		return sample[3];
	}
	
	public double getKP(){
		return KP;
	}
	
	public void setKP(double kp){
		this.KP = kp;
	}
	
	public double getKI(){
		return KI;
	}
	
	public void setKI(double ki){
		this.KI = ki;
	}
	
	public double getKD(){
		return KD;
	}
	
	public void setKD(double kd){
		this.KD = kd;
	}
	
	public double getCurrentError(){
		return error;
	}
	
	public void stopMotor(){
		leftMotor.flt();
		rightMotor.flt();
	}
}
