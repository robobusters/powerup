/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5057.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String driveStraightAuto = "Drive Straight Auto";
	private static final String FowardBackward = "Forward Backward";
	private static final String center = "Center Auto";
	private static final String left = "Left Auto";
	private static final String right = "Right Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	//ports
	final int leftDrivePort = 0;
	final int rightDrivePort = 1;
	final int liftPortL = 2;
	final int liftPortR = 3;
	final int intakePortL = 4;
	final int intakePortR = 5;
	
	//driveTrain
	DifferentialDrive chassis;
	Victor leftMotor = new Victor(leftDrivePort);
	Victor rightMotor = new Victor(rightDrivePort);
	JoystickLocations porting = new JoystickLocations();
	XboxController xbox = new XboxController(porting.joystickPort);
	DriveTrain dtr;
	
	//vision init. This is code we will use for the vision programming. Ignore for now
	VisionThread visionT;
	UsbCamera cam;
	double centerX=0.0;
	double centerX1;
	double centerX2;
	final Object imgLock = new Object();
	Relay LED;
	int relayPort = 3;
	
	//encoders and lift
	int directionLift = 0;
	boolean upButton = false;
	boolean downButton = false;
	int positionCase = 1;
	Victor liftLeft = new Victor(liftPortL);
	Victor liftRight = new Victor(liftPortR);
	Victor intakeLeft = new Victor(intakePortL);
	Victor intakeRight = new Victor(intakePortR);
	int encoderPort1 = 0;
	int encoderPort2 = 1;
	int encoderPort3 = 2;
	int encoderPort4 = 3;
	Encoder enc1 = new Encoder(encoderPort1, encoderPort2, false, Encoder.EncodingType.k4X);
	Encoder enc2 = new Encoder(encoderPort3, encoderPort4, false, Encoder.EncodingType.k4X);
	double distance1;
	double distance2;
	double rate1;
	double rate2;
	boolean direction1;
	boolean direction2;
	boolean stopped1;
	boolean stopped2;
	
	SpeedControllerGroup intake;
	
	//limit switches
	final int limitPortUp = 4;
	final int limitPortDown = 5;
	DigitalInput limUp = new DigitalInput(limitPortUp);
	DigitalInput limDown = new DigitalInput(limitPortDown);
	
	public float autoTimer;
	
	//things robot needs to do on startup
	public void robotInit() {
		//choose which autonomous to use
		m_chooser.addDefault("Drive Straight Auto", driveStraightAuto);
		m_chooser.addObject("Foward Backward", FowardBackward);
		m_chooser.addObject("Center Auto", center);
		m_chooser.addObject("Right Auto", right);
		m_chooser.addObject("Left Auto", left);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		rightMotor.setInverted(false);
		//setup drivetrain
		chassis = new DifferentialDrive(leftMotor, rightMotor);
		chassis.setExpiration(.1);
		chassis.setSafetyEnabled(false);
		dtr = new DriveTrain(chassis, xbox, porting);
		
		liftLeft.setInverted(true);
		
		intakeRight.setInverted(true);
		intakeRight.setSafetyEnabled(false);
		intakeLeft.setSafetyEnabled(false);
		intake = new SpeedControllerGroup(intakeLeft, intakeRight);
		
		//setup gyro
		dtr.gyro.calibrate();
		dtr.gyro.reset();
		
		autoTimer = System.nanoTime();
		
		//set up encoders
		enc1.setMaxPeriod(.1);
		enc1.setMinRate(10);
		enc1.setDistancePerPulse(0.71);
		enc1.setReverseDirection(false);
		enc1.setSamplesToAverage(7);
		enc1.reset();
		enc2.setMaxPeriod(.1);
		enc2.setMinRate(10);
		enc2.setDistancePerPulse(0.71);
		enc2.setReverseDirection(false);
		enc2.setSamplesToAverage(7);
		enc2.reset();
		
		//vision code init
		LED = new Relay(relayPort);
		LED.set(Relay.Value.kForward);
		cam= CameraServer.getInstance().startAutomaticCapture(0);//create new camera instance
		cam.setResolution(320*2,240*2);//control pixel size
		visionT=new VisionThread(cam, new GripPipeline(), pipeline -> {
			while(!VisionThread.interrupted()) {
			if (!pipeline.filterContoursOutput().isEmpty()) {//check to see if image is there. If image:
				Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));//left contour
				Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));//right contour
				synchronized(imgLock) {
            		centerX1 = r1.x + (r1.width / 2);//find x of left
            		centerX2 = r2.x + (r2.width/2);//find x or right
            		centerX = (centerX1 + centerX2)/2;//average them together
            	}SmartDashboard.putNumber("centerX", centerX);//publish to dashboard
			}else {//if image doesnt exist
				synchronized (imgLock) {
	    		centerX = -1;//center isn't available
	    		}
	    		SmartDashboard.putNumber("centerX", centerX);//publish
	    	}
			
			if(isAutonomous()){//if autonomous, allow camera to take lot of CPU
	        	Timer.delay(.01);
	        }
	        else{//if not auto, don't let camera run often
	            Timer.delay(0.1);
	        }
			
		}});
		visionT.start();
	}

	/**
	 * This autonomous. You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	int state = 1;
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		dtr.chassis.setSafetyEnabled(true);
		state = 1;
		//This is default code we use to choose an autonomous
	}

	/**This function is called periodically during autonomous.**/
	@Override
	public void autonomousPeriodic() { 
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		switch (m_autoSelected) {
			case FowardBackward://testing autonomous
				forwardBackward();
				break;
			case driveStraightAuto://nothing right now
				driveStraight();
				break;
			case center:
				if(gameData.length()>0) {
					if(gameData.charAt(0)=='L') {
						driveCenter(false);
					}else {
						driveCenter(true);
					}
				}
				break;
			case left:
				forwardBackward();
				/*if(gameData.length()>0) {
					if(gameData.charAt(0)=='L') {
						leftAuton(false);
					}else {
						leftAuton(true);
					}
				}*/
				break;
			case right:
				/*if(gameData.length()>0) {
					if(gameData.charAt(0)=='L') {
						rightAuton(false);
					}else {
						rightAuton(true);
					}
				}*/
				break;
			default:
				// Put default auto code here
				break;
			
		}Timer.delay(.005);
	}
	
	public void driveCenter(boolean isRight){
		switch(state){
		case 1:
			chassis.setSafetyEnabled(false);
			dtr.chassis.arcadeDrive(.58, 0);
			Timer.delay(1.75);
			state++;
			break;
		case 2:
			dtr.chassis.arcadeDrive(0,0);
			if(isRight){
				if(dtr.gyro.getAngle()>28){
					dtr.chassis.arcadeDrive(0,0);
					state++;
				}else{
					dtr.chassis.arcadeDrive(0,.59);
				}
			}else{
				if(dtr.gyro.getAngle()<-30){
					dtr.chassis.arcadeDrive(0,0);
					state++;
				}else{
					dtr.chassis.arcadeDrive(0,-.59);
				}
			}
			break;
		case 3:
			dtr.chassis.arcadeDrive(.75,0);
			Timer.delay(4);
			dtr.chassis.arcadeDrive(-.625,0);
			Timer.delay(1);
			dtr.chassis.arcadeDrive(.625,0);
			Timer.delay(1.5);
			dtr.chassis.arcadeDrive(-.625,0);
			Timer.delay(3);
			state++;
			break;
		case 4:
			dtr.chassis.arcadeDrive(0,0);
			chassis.setSafetyEnabled(true);
			state++;
			break;
		default:
			dtr.chassis.arcadeDrive(0,0);
			break;
		}
	}
	
	public void driveStraight() {
		switch(state) {
		case 1:
			chassis.setSafetyEnabled(false);
			dtr.chassis.arcadeDrive(.625, 0);
			state++;
			break;
		case 2:
			Timer.delay(5);
			state++;
			break;
		case 3:
			dtr.chassis.arcadeDrive(0, 0);
			dtr.chassis.setSafetyEnabled(true);
			state++;
			break;
		}
	}
	
	public void forwardBackward() {
		switch(state) {
		case 1:
			chassis.setSafetyEnabled(false);
			dtr.chassis.arcadeDrive(.75, 0);
			intake.set(-.15);
			state++;
			break;
		case 2:
			Timer.delay(2.5);
			dtr.chassis.arcadeDrive(0, 0);
			state++;
			break;
		case 3:
			state++;
			break;
		case 4:
			dtr.chassis.arcadeDrive(-.75, 0);
			state++;
			break;
		case 5:
			Timer.delay(2.5);
			state++;
			dtr.chassis.arcadeDrive(0, 0);
			intake.set(0);
			break;
		}
	}
	
	public void testMethods() {
		switch(state) {
		case 1:
			findTime(100);
			state++;
			break;
		case 2:
			if(driveDistance())state++;
			break;
		}
	}
	
	/**This function is called periodically during operator control.*/
	double intakeSpeed=.45;
	double outtakeSpeed=1;
	@Override
	public void teleopPeriodic() {
		dtr.chassis.setSafetyEnabled(true);
		dtr.changeDrive();
		dtr.updateAxes();
		LED.set(Relay.Value.kForward);
		getEnc();
		buttons();

		if(xbox.getRawAxis(porting.lTrigger)>.2) {
			intake.set(intakeSpeed*-xbox.getTriggerAxis(Hand.kLeft));
		}else if (xbox.getRawAxis(porting.rTrigger)>.2) {
			intake.set(outtakeSpeed*xbox.getTriggerAxis(Hand.kRight));
		}
		else
			intake.set(0);
	}
	
	/**This function is called periodically during test mode**/
	@Override
	public void testPeriodic() {}
	
	double firstAngle=4*4;
	double secondAngle=7*4;
	double dropSpeed = .25;
	double liftSpeed = .75;
	double idleSpeed = .25;
	public void buttons() {
		if(downButton == false && xbox.getRawButton(porting.butLBumper) == true)
		{
			if(positionCase != 1)
				positionCase--;
			directionLift = 0;
		}else if(upButton == false && xbox.getRawButton(porting.butRBumper) == true)
		{
			if(positionCase != 5)
				positionCase++;
			directionLift = 1;
		}
		SmartDashboard.putNumber("positionCase", positionCase);

		if(directionLift == 0)
		{
			switch(positionCase) {
			case 0: 
				liftLeft.set(0);
				liftRight.set(0);
				break;
			case 1:
				if(enc1.getDistance() > 0 && limDown.get() == false) 
					{
					liftLeft.set(-dropSpeed);
					liftRight.set(-dropSpeed);
					}
					
				else
					{
					liftLeft.set(0);
					liftRight.set(0);
					}
					break;
			case 2:
				if(enc1.getDistance() > firstAngle && limDown.get() == false)
					{
					liftLeft.set(-dropSpeed);
					liftRight.set(-dropSpeed);
					}
				else
					{
					liftLeft.set(idleSpeed);
					liftRight.set(idleSpeed);
					}
					break;
			case 3:
				if(enc1.getDistance() > secondAngle && limDown.get() == false)
					{
					liftLeft.set(-dropSpeed);
					liftRight.set(-dropSpeed);
					}
				else
					{
					liftLeft.set(idleSpeed);
					liftRight.set(idleSpeed);
					}
				break;
			case 4:
				if(enc1.getDistance() > 110*4 && limDown.get() == false)
					{	
					liftLeft.set(-0.25);
					liftRight.set(-0.25);
					}
				else
					{
					liftLeft.set(0);
					liftRight.set(0);
					}
				break;
			}
		}
		if(directionLift == 1)
		{
			switch(positionCase) {
			case 2:
				if(enc1.getDistance() < firstAngle && limUp.get() == false)
					{
					liftLeft.set(liftSpeed);
					liftRight.set(liftSpeed);
					}
				else
					{
					liftLeft.set(idleSpeed);
					liftRight.set(idleSpeed);
					}
				break;
			case 3:
				if(enc1.getDistance() < secondAngle && limUp.get() == false)
					{
					liftLeft.set(liftSpeed);
					liftRight.set(liftSpeed);
					}
				else
					{
					liftLeft.set(idleSpeed);
					liftRight.set(idleSpeed);
					}
				break;
			case 4:
				if(enc1.getDistance() < 110*4 && limUp.get() == false)
					{
					liftLeft.set(0.75);
					liftRight.set(0.75);
					}
				else
					{
					liftLeft.set(0);
					liftRight.set(0);
					}
				break;
			case 5:
				if(enc1.getDistance() < 180*4 && limUp.get() == false)
					{
					liftLeft.set(0.75);
					liftRight.set(0.75);
					}
				else
					{
					liftLeft.set(0);
					liftRight.set(0);
					}
				break;
			case 6:
				liftLeft.set(0);
				liftRight.set(0);
				break;
			}
		}
		downButton = xbox.getRawButton(porting.butLBumper);
		upButton = xbox.getRawButton(porting.butRBumper);		
	}
	
	public void getEnc() {

		distance1 = enc1.getDistance();
		rate1 = enc1.getRate();
		direction1 = enc1.getDirection();
		stopped1 = enc1.getStopped();
		SmartDashboard.putNumber("distance1", distance1);
		SmartDashboard.putBoolean("direction1",direction1);
		SmartDashboard.putNumber("rate1",rate1);
		SmartDashboard.putBoolean("stopped1?",stopped1);
		
		distance2 = enc2.getDistance();
		rate2 = enc2.getRate();
		direction2 = enc2.getDirection();
		stopped2 = enc2.getStopped();
		SmartDashboard.putNumber("distance2", distance2);
		SmartDashboard.putBoolean("direction2",direction2);
		SmartDashboard.putNumber("rate2",rate2);
		SmartDashboard.putBoolean("stopped2?",stopped2);
	}
	
	public long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }
	
	public boolean driveDistance() {
		if(autoTimer<System.nanoTime()) {
			dtr.chassis.arcadeDrive(0, 0);
			return true;
		}
		dtr.chassis.arcadeDrive(.625, 0);
		return false;
	}
	public double findTime(double distanceCm) {
		double averageSpeed = (290-198);//cm/s
		double time = averageSpeed/distanceCm;
		autoTimer = futureTime((long)time+'f');
		return time;
	}
	
	public void leftAuton(boolean isRight) {
		if(isRight) {
			switch(state) {
			case 1:
				findTime(550);
				state++;
				break;
			case 2:
				if(driveDistance()) {
					state++;
				}break;
			case 3:
				break;
			}
		}else {
			switch(state) {
			case 1:
				if(dtr.turnRight(10)) {
					state++;
				}break;
			case 2:
				findTime(375);
				state++;
				break;
			case 3:
				if(driveDistance()) {
					state++;
				}break;
			case 4:
				findTime(-15);
				state++;

				break;
			case 5:
				if(driveDistance()) {
					state++;
				}
				break;
			case 6:
				if(enc1.getDistance() < 20*4 && limUp.get() == false)
				{
				liftLeft.set(0.5);
				liftRight.set(0.5);
				}
			else
				{
				liftLeft.set(0);
				liftRight.set(0);
				state++;
				}
				break;
			case 7:
				intake.set(-1);
				if(autoTimer < System.nanoTime()) {
					state++;
					intake.set(0);}
				break;
			case 8:
				//drop arm
				if(enc1.getDistance() > 20*4 && limDown.get() == false)
				{
				liftLeft.set(-0.5);
				liftRight.set(-0.5);
				}
			else
				{
				liftLeft.set(0);
				liftRight.set(0);
				state++;
				autoTimer=futureTime(1.5f);
				}
				break;
			case 9:
				state++;
				break;
			case 10:
				state++;
				break;
			case 11:
				if(dtr.turnLeft(90)) {
					findTime(225);
					state++;
				}
				break;
			case 12:
				if(driveDistance()) {
					state++;
				}break;
			case 13:
				if(dtr.turnRight(0)) {
					state++;
					findTime(50);
				}
				break;
			case 14:
				if(driveDistance()) {
					state++;
				}break;
			}
		}
	}

	public void rightAuton(boolean isRight) {
		if(!isRight) {
			switch(state) {
			case 1:
				findTime(550);
				state++;
				break;
			case 2:
				if(driveDistance()) {
					state++;
				}break;
			case 3:
				break;
			}
		}else {
			switch(state) {
			case 1:
				state++; break;
			case 2:
				findTime(375);
				state++;
				break;
			case 3:
				if(driveDistance()) {
					state++;
				}break;
			case 4:
				findTime(-15);
				state++;
				break;
			case 5:
				if(driveDistance()) {
					state++;
				}
				break;
			case 6:
				//lift arm
				state++;
				break;
			case 7:
				//out take
				state++;
				break;
			case 8:
				//drop arm
				state++;
				break;
			case 9:
				state++;
				break;
			case 10:
				state++;
				break;
			case 11:
				if(dtr.turnRight(90)) {
					findTime(225);
					state++;
				}
				break;
			case 12:
				if(driveDistance()) {
					state++;
				}break;
			case 13:
				if(dtr.turnLeft(0)) {
					state++;
					findTime(50);
				}
				break;
			case 14:
				if(driveDistance()) {
					state++;
				}break;
			}
		}
	}
	
	public void centerAuton(boolean isRight) {
		if(isRight) {
			switch(state) {
			case 1:
				findTime(33);
				state++;
				break;
			case 2:
				if(driveDistance()) {
					state++;
				}
				break;
			case 3:
				if(dtr.turnRight(53)) {
					state++;
				}
				break;
			case 4:
				findTime(350);
				state++;
				break;
			case 5:
				if(driveDistance()) {
					state++;
				}
				break;
			case 6:
				if(dtr.turnLeft(0)) {
					state++;
					findTime(10);
				}
				break;
			case 7:
				if(driveDistance()) {
					state++;
				}break;
			case 8:
				//lift arm
				state++;
				break;
			case 9:
				//deposit
				state++;
				break;
			case 10:
				//drop arm
				state++;
				break;
			case 11:
				if(dtr.turnRight(90)) {
					findTime(225);
					state++;
				}
				break;
			case 12:
				if(driveDistance()) {
					state++;
				}break;
			case 13:
				if(dtr.turnLeft(0)) {
					state++;
					findTime(50);
				}
				break;
			case 14:
				if(driveDistance()) {
					state++;
				}break;
			}
		}else {
			switch(state) {
			case 1:
				findTime(33);
				state++;
				break;
			case 2:
				if(driveDistance()) {
					state++;
				}
				break;
			case 3:
				if(dtr.turnLeft(53)) {
					state++;
				}
				break;
			case 4:
				findTime(350);
				state++;
				break;
			case 5:
				if(driveDistance()) {
					state++;
				}
				break;
			case 6:
				if(dtr.turnRight(0)) {
					state++;
					findTime(10);
				}
				break;
			case 7:
				if(driveDistance()) {
					state++;
				}break;
			case 8:
				//lift arm
				state++;
				break;
			case 9:
				//deposit
				state++;
				break;
			case 10:
				//drop arm
				state++;
				break;
			case 11:
				if(dtr.turnLeft(90)) {
					findTime(225);
					state++;
				}
				break;
			case 12:
				if(driveDistance()) {
					state++;
				}break;
			case 13:
				if(dtr.turnRight(0)) {
					state++;
					findTime(50);
				}
				break;
			case 14:
				if(driveDistance()) {
					state++;
				}break;
			}
		}
	}

	
	//temporary autonomous to drive with sonar
	public void autoSonar() {
		dtr.chassis.setSafetyEnabled(true);//allow auto to happen
		dtr.getHeading();
		dtr.getDistance();
		dtr.getAccel();
		dtr.updateAxes();
		if(dtr.currentDistance1>18) {//if the robot is more than 18 inches from the wall
			SmartDashboard.putNumber("AutoDriveOn",0);//say robot  moving
			dtr.chassis.arcadeDrive(.5, 0);
		}else {
			SmartDashboard.putNumber("AutoDriveOn", 1);//say robot not moving
			dtr.chassis.arcadeDrive(0, 0);
		}
		
		SmartDashboard.putNumber("leftSpeed", leftMotor.get()*-1);
		SmartDashboard.putNumber("rightSpeed", rightMotor.get());
	}
}
