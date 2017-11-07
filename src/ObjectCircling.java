
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

	final static double RADIUS = .0275; // RADIUS of the tires in meters
	final static double PI = 3.141592653589793;
	final static float SONAR_OFFSET = .022f; // how far the sonar is from front
												// of robut
	final static double AXLE_LENGTH = .122;
	
	static double mOrientation = PI/ 2.0;
	static double mLeftX = 0.0;
	static double mLeftY = 0.0;
	static double mRightX = 0.0;
	static double mRightY = 0.0;
	static boolean mHasExitedHitpoint = false;
	static double[] mHitpoint = new double[2];
	static EV3MediumRegulatedMotor left;
	static EV3MediumRegulatedMotor right;
	static SensorMode touchLeft;
	static SensorMode touchRight;
	static SensorMode sonic;
	static float[] touchLeftSample;
	static float[] touchRightSample;
	static float[] sonicSample;

	

	public static void main(String[] args) {

		left = new EV3MediumRegulatedMotor(MotorPort.A);
		right = new EV3MediumRegulatedMotor(MotorPort.D);
		left.synchronizeWith(new EV3MediumRegulatedMotor[] { right });
		EV3TouchSensor touchLeftSensor = new EV3TouchSensor(SensorPort.S3);
		EV3TouchSensor touchRightSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		touchLeft = touchLeftSensor.getTouchMode();
		touchRight = touchRightSensor.getTouchMode();
		sonic = (SensorMode) ultraSensor.getDistanceMode();
		touchLeftSample = new float[touchLeft.sampleSize()];
		touchRightSample = new float[touchRight.sampleSize()];
		sonicSample = new float[sonic.sampleSize()];
		// start and head forward
		Sound.beep();
		Button.ENTER.waitForPressAndRelease();
		System.out.println("Moving forward");
		long timestamp = System.nanoTime();
		left.startSynchronization();
		right.forward();
		left.forward();
		left.endSynchronization();

		// stop and beep when you hit a wall
		touchLeft.fetchSample(touchLeftSample, 0);
		touchRight.fetchSample(touchRightSample, 0);
		while (touchLeftSample[0] == 0 || touchRightSample[0] == 0) {
			touchLeft.fetchSample(touchLeftSample, 0);
			touchRight.fetchSample(touchRightSample, 0);
		}
		Sound.beep();
		left.startSynchronization();
		right.stop();
		left.stop();
		left.endSynchronization();
		updateCoordsLinear(timestamp);

		// back up 15cm
		Sound.beep();
		System.out.println("Moving Backwards");
		move(-.15f, false);
		mHitpoint = getCenterCoords();
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

		rotateAngle((float) (-PI/2.0));
		Sound.beep();

		followWall();
	}

	private static void followWall() {
		// wall following (Bang Bang)
		float setDistance = .10f;
		float initspeed = 180f;
		float error = 0f;
		float newerror = 0f;
		float errordiff = 0f;
		float setbuffer = 0.05f;
		float terminatediff = 0.4f;
		float distanceTraveled = 0f;
		float adjustAngle = 0f;
		float ssample;

		float infinity = .30f;
		long travelTime = 250000000; // in nanoseconds
		long timestamp;
		boolean forever = true;

		left.startSynchronization();
		right.forward();// left wheel
		left.forward();// right wheel
		left.endSynchronization();
		ssample = fetchSonicSample();
		error = ssample - setDistance;

		touchLeft.fetchSample(touchLeftSample, 0);
		touchRight.fetchSample(touchRightSample, 0);

		while (forever) {

			newerror = ssample - setDistance;
			errordiff = newerror - error; // if positive, error increase
			// System.out.print("E " + newerror + " " + errordiff + " ");

            //according to the error difference, adjust the angle with one wheel set to speed 0
			if ( mHasExitedHitpoint && (Math.abs(getCenterCoords()[0] - mHitpoint[0]) < .15) 
					&&  (Math.abs(getCenterCoords()[1] - mHitpoint[0]) < .15)){//end of the wall, break loopn
				break;
			}else {
				if(newerror< -1*setbuffer || newerror> setbuffer){//if drifting left from the offset turn right
					adjustAngle = calculateAngle(error, newerror, distanceTraveled );
					rotateAngle(adjustAngle);
				}

			}
			// } else if(abs(errordiff) > setbuffer){
			// //adjust angle
			// //calculate distance traveled
			// distanceTraveled = (float) ( initspeed * travelTime*0.001 * (PI /
			// 360) * RADIUS); //m
			// System.out.print("D " + distanceTraveled + " " );//15.118915
			// adjustAngle = calculateAngle(error, newerror,distanceTraveled );
			// rotateAngle(adjustAngle, left, right);
			//
			// }

			timestamp = System.nanoTime() + travelTime;
			while (System.nanoTime() < timestamp) {
				touchLeft.fetchSample(touchLeftSample, 0);
				touchRight.fetchSample(touchRightSample, 0);
				if (touchLeftSample[0] != 0 || touchRightSample[0] != 0) {
					System.out.println("Collision detected");

					move( -.15f, false);
                    ssample = fetchSonicSample();
                    error = ssample - setDistance;
					
					rotateAngle( (float) (-PI/4.0));
					move( .10f, false);
                    ssample = fetchSonicSample();
                    newerror = ssample - setDistance;

					left.startSynchronization();
					right.forward();// left wheel
					left.forward();// right wheel
					left.endSynchronization();
					break;
				}
			}
			error = newerror;
			left.setSpeed(initspeed);
			right.setSpeed(initspeed);

			ssample = fetchSonicSample();
			touchLeft.fetchSample(touchLeftSample, 0);
			touchRight.fetchSample(touchRightSample, 0);
			updateCoordsLinear(timestamp);
		}
		left.startSynchronization();
		right.stop();
		left.stop();
		left.endSynchronization();

		System.out.println("Going home!");
		//rotate to face home, go home
		rotateAngle((float) (-mOrientation - PI/2.0));
		float distanceToHome = (float) sqrt(mHitpoint[0] * mHitpoint[0] +  mHitpoint[1] * mHitpoint[1]);
		move(distanceToHome, false);
	}

	private static void move(float distanceToGo, boolean wallReturn) {
		move(distanceToGo, 180, wallReturn);
	}

	private static void move(float distanceToGo, int speed, boolean wallReturn) {
		long timestamp = System.nanoTime();
		left.setSpeed(speed);
		right.setSpeed(speed);
		double numRotations = (distanceToGo / (RADIUS * 2.0 * PI));
		int angle = (int) (360.0 * numRotations);
		// System.out.println("moving wheels " + angle + " degrees ");

		left.startSynchronization();
		left.rotate(angle, true);
		right.rotate(angle, true);
		left.endSynchronization();
		touchLeft.fetchSample(touchLeftSample, 0);
		touchRight.fetchSample(touchRightSample, 0);
		if (distanceToGo < 0) {
			while (left.isMoving()) {
			}
		} else {
			while (left.isMoving()) {
				touchLeft.fetchSample(touchLeftSample, 0);
				touchRight.fetchSample(touchRightSample, 0);
				if (touchLeftSample[0] != 0 || touchRightSample[0] != 0) {
					System.out.println("Collision!");
					Sound.beep();
					move(-.15f, false);

					rotateAngle((float) (-PI/2.0));

					followWall();
					break;
				}
			}
		}
		updateCoordsLinear(timestamp);
	}

	private static void rotateAngle(float angle) {
		assert (right.getRotationSpeed() == 0 || left.getRotationSpeed() == 0);
		long initTime = System.nanoTime();
		long timeToRotate;
		float desiredAngularVelocity;
		int wheelRotationSpeedDegrees, RightwheelRotationSpeedDegrees, LeftwheelRotationSpeedDegrees;
		float wheelRotationSpeedRadians;

		// System.out.print((int) (angle * 180.0f / PI) + "degrees");
		RightwheelRotationSpeedDegrees = right.getRotationSpeed();

		LeftwheelRotationSpeedDegrees = left.getRotationSpeed();
		
		if (angle > 0) {	//turning left

			wheelRotationSpeedDegrees = right.getRotationSpeed();

			if (!right.isMoving()) { // sammy is stationary
				// System.out.println("stationary left turn");
				wheelRotationSpeedDegrees = 180;
				right.setSpeed(wheelRotationSpeedDegrees);
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( angle / desiredAngularVelocity * 1000000000.0) + System.nanoTime(); 
				right.forward();
				while (System.nanoTime() < timeToRotate) {
					right.forward();
				}
				right.stop();
				
			} else { //sammie was originally moving
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( angle / desiredAngularVelocity * 1000000000.0) + System.nanoTime(); 
				left.stop();
				while (System.nanoTime() < timeToRotate) {
				}
				left.forward();
			}

			mRightX = mLeftX + AXLE_LENGTH * (Math.cos(angle + mOrientation - PI/2.0));
			mRightY = mLeftY + AXLE_LENGTH * (Math.sin(angle + mOrientation - PI/2.0));
			
			
		} else {	//turning right
			wheelRotationSpeedDegrees = left.getRotationSpeed();

			if (!left.isMoving()) { // sammy is stationary
				// System.out.println("stationary right turn");
				wheelRotationSpeedDegrees = 180;
				left.setSpeed(wheelRotationSpeedDegrees);
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity * 1000000000.0)  + System.nanoTime(); 
				// System.out.print("T " + timeToRotate + "  ");
				left.forward();
				while (System.nanoTime() < timeToRotate) {
					left.forward();
				}
				left.stop();

			} else {

				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) (-angle / desiredAngularVelocity * 1000000000.0)  + System.nanoTime(); 
				right.stop();
				while (System.nanoTime() < timeToRotate) {
				}
				right.forward();
			}
			mLeftX = mRightX + AXLE_LENGTH * (Math.cos(angle + mOrientation - PI/2.0));
			mLeftY = mRightY + AXLE_LENGTH * (Math.sin(angle + mOrientation - PI/2.0));
		}
		left.setSpeed(wheelRotationSpeedDegrees);
		right.setSpeed(wheelRotationSpeedDegrees);
		mOrientation += angle;
		if (mOrientation > (2.0 * PI)) {
			mOrientation -= 2.0 * PI;
		} else if (mOrientation < (-2.0 * PI)) {
			mOrientation += 2.0 * PI;
		}
	}

	// takes in two sonar readings and the distance traveled between those two
	// readings
	// outputs the angle of attack to object detected by sonar in radians
	// positive values mean going towards object
	private static float calculateAngle(float sonar0, float sonar1, float distanceTravelled) {
		float angle;
		float unitAngle = (float) (10 * (PI / 180));
		float maxAngle = (float) (20 * (PI / 180));
		float sonarscaler = 100f;

		if(sonar1>0){//turn left
			if(unitAngle*(sonar1*sonarscaler) > maxAngle){
				angle=maxAngle;
			}else{
				angle= unitAngle*(sonar1*sonarscaler);
			}
		}else{//turn right
			if(unitAngle*(-1*sonar1*sonarscaler) > maxAngle){
				angle= -maxAngle;
			}else{
				angle= -unitAngle*(sonar1*sonarscaler);
			}
		}
		
		// angle= (float) Math.atan((sonar0 - sonar1)/distanceTravelled);
		return angle;
	}


	// fetch 3 sonic sample. If they are in an acceptable range, return avarage.
	// If not, take again
	private static float fetchSonicSample() {
		int samplesize = 3;
		boolean acceptable;
		float s[] = new float[samplesize];
		float sum, ave;

		for (int i = 0; i < samplesize; i++) {
			sonic.fetchSample(sonicSample, 0);
			s[i] = sonicSample[0];
		}
		acceptable = checkAcceptable(s);

		while (!acceptable) {
			for (int i = 0; i < samplesize; i++) {
				sonic.fetchSample(sonicSample, 0);
				s[i] = sonicSample[0];
			}
			acceptable = checkAcceptable(s);

		}
		sum = 0;
		for (int i = 0; i < samplesize; i++) {
			sum = sum + s[i];
		}
		ave = sum / samplesize;
		return ave;

	}

	private static boolean checkAcceptable(float s[]) {
		float max, min, maxdiff, tol;
		boolean b = true;
		tol = (float) 1.0;
		max = s[0];
		min = s[0];
		for (int i = 0; i < s.length; i++) {
			// find min
			if (s[i] < min) {
				min = s[i];
			}
			// find max
			if (s[i] > max) {
				max = s[i];
			}
		}
		maxdiff = max - min;
		if (maxdiff > tol)
			b = false;
		return b;
	}


	
	//use this after moving forward in a straight line
	private static void updateCoordsLinear(long previousTime) {
		if (!mHasExitedHitpoint && (mHitpoint[0] != 0.0 || mHitpoint[1] != 0.0)){
			if(Math.abs(mHitpoint[0] - getCenterCoords()[0]) < .2 && Math.abs(mHitpoint[1] - getCenterCoords()[1]) < .2){
				mHasExitedHitpoint = true;
				System.out.println("Exited Hit Bubble");
				Sound.beep();
			}
		}
		
		int RightwheelRotationSpeedDegrees = right.getRotationSpeed();
		int LeftwheelRotationSpeedDegrees = left.getRotationSpeed();
		assert (RightwheelRotationSpeedDegrees == LeftwheelRotationSpeedDegrees);
		double linearSpeed = (LeftwheelRotationSpeedDegrees  * PI / 180.0) * RADIUS;
		double distance = ((double) (System.nanoTime() - previousTime) / 1000000000.0 ) * linearSpeed;
		
		mLeftX += distance * Math.cos(mOrientation);
		mRightX += distance * Math.cos(mOrientation);

		mLeftY += distance * Math.sin(mOrientation);
		mRightY += distance * Math.sin(mOrientation);
		
		if(mHasExitedHitpoint){
			System.out.println("Coords: " + String.format("%.3g%n", getCenterCoords()[0]) + ", " + String.format("%.3g%n", getCenterCoords()[1]));
		}
		//if it hasn't left the initialized hitpoint yet
	}
	
	private static double[] getCenterCoords(){
		return new double[]{(mLeftX + mRightX)/2.0, (mLeftY + mRightY)/2.0 };
	}

}
