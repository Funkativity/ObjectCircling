/****
 * Author: Yan Ren, Victor Murta
 */
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import static java.lang.Math.*;



	
public class ObjectCircling {
	
	final static double RADIUS= .0275; //RADIUS of the tires in meters
	final static double PI = 3.141592653589793;
	final static float SONAR_OFFSET = .022f; //how far the sonar is from front of robut
	final static double AXLE_LENGTH = .122;
	// static double mDisplacement = 0.0;
	static double mOrientation = 0.0;
	static EV3MediumRegulatedMotor left;
	static EV3MediumRegulatedMotor right;
	static SensorMode touchLeft;
	static SensorMode touchRight;
	static SensorMode sonic;
	static float[] touchLeftSample;
	static float[] touchRightSample;
	static float[] sonicSample; 
	
	public static void main(String[] args) {
		
		left= new EV3MediumRegulatedMotor(MotorPort.A);
		right = new EV3MediumRegulatedMotor(MotorPort.D);
		left.synchronizeWith(new EV3MediumRegulatedMotor[] {right});
		EV3TouchSensor touchLeftSensor = new EV3TouchSensor(SensorPort.S3);
		EV3TouchSensor touchRightSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		touchLeft = touchLeftSensor.getTouchMode();
		touchRight = touchRightSensor.getTouchMode();
		sonic = (SensorMode) ultraSensor.getDistanceMode();
		touchLeftSample = new float[touchLeft.sampleSize()];
		touchRightSample = new float[touchRight.sampleSize()];
		sonicSample = new float[sonic.sampleSize()];
		//start and head forward
		Sound.beep();
		Button.ENTER.waitForPressAndRelease();
		System.out.println("Moving forward");
		left.startSynchronization();
		right.forward();
		left.forward();
		left.endSynchronization();
		
		//stop and beep when you hit a wall
		touchLeft.fetchSample(touchLeftSample, 0);
		touchRight.fetchSample(touchRightSample, 0);
		while(touchLeftSample[0] == 0 || touchRightSample[0] == 0){
			touchLeft.fetchSample(touchLeftSample, 0);
			touchRight.fetchSample(touchRightSample, 0);
		}
		Sound.beep();
		left.startSynchronization();
		right.stop();
		left.stop();
		left.endSynchronization();

		//back up 15cm
		Sound.beep();
		System.out.println("Moving Backwards");
		move(-.15f, false);
//		Button.ENTER.waitForPressAndRelease();
//		double numRotations = ( .15 / (RADIUS * 2 * PI));
//		int angle = (int) (-360.0 * numRotations);
//		left.startSynchronization();
//		left.rotate(angle, false);
//		right.rotate(angle, false);
//		left.endSynchronization();
//		Button.ENTER.waitForPressAndRelease();
		
		//turn right 
		System.out.println("turn right");
		rotateAngle((float) (PI/2.0));
		Sound.beep();
		
		followWall();
		

	}
	
	private static void followWall() {
		//wall following (Bang Bang)
		float setDistance = .10f;
		float initspeed = 180f;
		float error = 0f;
		float newerror = 0f;
		float errordiff = 0f;
		float setbuffer = 0.05f;
		float terminatediff = 0.4f;
		float distanceTraveled = 0f;
		float adjustAngle = 0f;

		float infinity = .30f;
		long travelTime = 250000000; //in nanoseconds
		long timestamp;
		boolean forever = true;
		
		left.startSynchronization();
		right.forward();//left wheel
		left.forward();//right wheel
		left.endSynchronization();
		sonic.fetchSample(sonicSample, 0);
		error = sonicSample[0] - setDistance;
		
		touchLeft.fetchSample(touchLeftSample, 0);
		touchRight.fetchSample(touchRightSample, 0);
		while(forever){
			
			newerror = sonicSample[0] - setDistance;			
			errordiff = newerror - error; // if positive, error increase
			System.out.print("E " + newerror + " " + errordiff+ " ");

			//according to the error difference, adjust the angle with one wheel set to speed 0
			if ( abs(errordiff) > terminatediff || newerror > infinity ){//end of the wall, break loop
				
				break;
			}else {
				if(newerror< -1*setbuffer || newerror> setbuffer){//if drifting left from the offset turn right
					adjustAngle = calculateAngle(error, newerror,distanceTraveled );
					rotateAngle(adjustAngle);	
				}

			}
//			} else if(abs(errordiff) > setbuffer){
//				//adjust angle
//				//calculate distance traveled
//				distanceTraveled = (float) ( initspeed * travelTime*0.001 * (PI / 360) * RADIUS); //m
//				System.out.print("D " + distanceTraveled + " " );//15.118915
//				adjustAngle = calculateAngle(error, newerror,distanceTraveled );
//				rotateAngle(adjustAngle, left, right);
//				
//			}

			
			timestamp = System.nanoTime() + travelTime;
			while(System.nanoTime() < timestamp) {
				touchLeft.fetchSample(touchLeftSample, 0);
				touchRight.fetchSample(touchRightSample, 0);
				if(touchLeftSample[0] != 0 || touchRightSample[0] != 0){
					System.out.println("Collision detected");
					move( -.15f, false);
					sonic.fetchSample(sonicSample, 0);
					error = sonicSample[0] - setDistance;	
					
					rotateAngle( (float) (PI/6.0));
					move( .10f, false);
					sonic.fetchSample(sonicSample, 0);
					newerror = sonicSample[0] - setDistance;			
					left.startSynchronization();
					right.forward();//left wheel
					left.forward();//right wheel
					left.endSynchronization();
					break;
				}
			}
			error = newerror;
			left.setSpeed(initspeed);
			right.setSpeed(initspeed);
			
			sonic.fetchSample(sonicSample, 0);
			touchLeft.fetchSample(touchLeftSample, 0);
			touchRight.fetchSample(touchRightSample, 0);
		}
		left.startSynchronization();
		right.stop();
		left.stop();
		left.endSynchronization();
		Sound.beep();
		
		move(.1f, true);	
		System.out.println("Final orientation: " + (int) (mOrientation * 180.0 / PI));
		//turn to face forward
		
		rotateAngle((float) -mOrientation);
		Sound.beepSequenceUp();
		
		//move 0.75m 
		move(.75f, true);
		Button.ENTER.waitForPressAndRelease();
	}
	
	private static void move(float distanceToGo, boolean wallReturn) {
		move(distanceToGo, 180, wallReturn);
	}
	
	private static void move(float distanceToGo, int speed, boolean wallReturn) {
		left.setSpeed(speed);
		right.setSpeed(speed);
		double numRotations = ( distanceToGo / (RADIUS * 2.0 * PI));
		int angle = (int) (360.0 * numRotations);
		System.out.println("moving wheels " + angle + " degrees ");
		
		left.startSynchronization();
		left.rotate(angle, true);
		right.rotate(angle, true);
		left.endSynchronization();
		touchLeft.fetchSample(touchLeftSample, 0);
		touchRight.fetchSample(touchRightSample, 0);
		if (distanceToGo < 0) {
			while(left.isMoving())  {
			}
		} else {
			while(left.isMoving()) {
				touchLeft.fetchSample(touchLeftSample, 0);
				touchRight.fetchSample(touchRightSample, 0);
				if (touchLeftSample[0] != 0 || touchRightSample[0] != 0) {
					System.out.println("Collision!");
					Sound.beep();
					move(-.15f, false);
					rotateAngle((float) (PI/2.0));
					followWall();
					break;
				}
			}
		}
	}


	private static void rotateAngle(float angle) {
		assert(right.getRotationSpeed() == 0 || left.getRotationSpeed() == 0);
		long initTime = System.nanoTime();
		long timeToRotate;
		float desiredAngularVelocity;
		int wheelRotationSpeedDegrees,RightwheelRotationSpeedDegrees,LeftwheelRotationSpeedDegrees;
		float wheelRotationSpeedRadians;
		
		System.out.print((int)(angle * 180.0f/PI) + "degrees" );
		RightwheelRotationSpeedDegrees = right.getRotationSpeed();
		LeftwheelRotationSpeedDegrees = right.getRotationSpeed();
		
		if (angle < 0) {	//turning left
			
			wheelRotationSpeedDegrees = right.getRotationSpeed();
			
			if (!right.isMoving()) { //sammy is stationary
//				System.out.println("stationary left turn");
				wheelRotationSpeedDegrees = 180;
				right.setSpeed(wheelRotationSpeedDegrees);
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity * 1000000000.0) + System.nanoTime(); 
				right.forward();
				while(System.nanoTime() < timeToRotate) {
					right.forward();
				}
				right.stop();
				
			} else { //sammie was originally moving
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity * 1000000000.0) + System.nanoTime(); 
				left.stop();
				while(System.nanoTime() < timeToRotate) {
				}
				left.forward();
			}
			
			
		} else {	//turning right
			wheelRotationSpeedDegrees = left.getRotationSpeed();
			
			if (!left.isMoving()) { //sammy is stationary 
				// System.out.println("stationary right turn");
				wheelRotationSpeedDegrees = 180;
				left.setSpeed(wheelRotationSpeedDegrees);
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( angle / desiredAngularVelocity * 1000000000.0)  + System.nanoTime(); 
				// System.out.print("T " + timeToRotate + "  ");
				left.forward();
				while(System.nanoTime() < timeToRotate) {
					left.forward();
				}
				left.stop();
				
			} else {
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) (angle / desiredAngularVelocity * 1000000000.0)  + System.nanoTime(); 
				right.stop();
				while(System.nanoTime() < timeToRotate) {
				}
				right.forward();
			}
		}
		left.setSpeed(wheelRotationSpeedDegrees);
		right.setSpeed(wheelRotationSpeedDegrees);
		mOrientation += angle;
		if (mOrientation > (2.0 * PI)) {
			mOrientation -= PI;
		}
	}
	
	//takes in two sonar readings and the distance traveled between those two readings
	//outputs the angle of attack to object detected by sonar in radians
	//positive values mean going towards object
	private static float calculateAngle(float sonar0, float sonar1, float distanceTravelled) {
		float angle;
		float unitAngle = (float) (10*(PI/180));
		float maxAngle = (float) (30*(PI/180));
		float sonarscaler = 100f;
		if(sonar1>0){//turn left
			if(unitAngle*(sonar1*sonarscaler) > maxAngle){
				angle=maxAngle*-1;
			}else{
				angle= unitAngle*(sonar1*sonarscaler) *-1 ;
			}
		}else{//turn right
			if(unitAngle*(-1*sonar1*sonarscaler) > maxAngle){
				angle=maxAngle;
			}else{
				angle= unitAngle*(sonar1*sonarscaler) *-1 ;
			}
		}
		//angle= (float) Math.atan((sonar0 - sonar1)/distanceTravelled);
		return angle;
	}
	

}
