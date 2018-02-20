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
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
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
	Spark leftMotor = new Spark(leftDrivePort);
	Spark rightMotor = new Spark(rightDrivePort);
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
	int positionCase = 0;
	Victor liftLeft = new Victor(liftPortL);
	Victor liftRight = new Victor(liftPortR);
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
	
	//limit switches
	final int limitPortUp = 4;
	final int limitPortDown = 5;
	DigitalInput limUp = new DigitalInput(limitPortUp);
	DigitalInput limDown = new DigitalInput(limitPortDown);
	
	//things robot needs to do on startup
	public void robotInit() {
		//choose which autonomous to use
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//setup drivetrain
		chassis = new DifferentialDrive(leftMotor, rightMotor);
		chassis.setExpiration(.1);
		chassis.setSafetyEnabled(false);
		dtr = new DriveTrain(chassis, xbox, porting);
		
		//setup gyro
		dtr.gyro.calibrate();
		dtr.gyro.reset();
		enc1.setMaxPeriod(.1);
		enc1.setMinRate(10);
		enc1.setDistancePerPulse(0.71);
		enc1.setReverseDirection(true);
		enc1.setSamplesToAverage(7);
		enc1.reset();
		
		enc2.setMaxPeriod(.1);
		enc2.setMinRate(10);
		enc2.setDistancePerPulse(0.71);
		enc2.setReverseDirection(true);
		enc2.setSamplesToAverage(7);
		enc2.reset();
		
		//vision code init
		LED = new Relay(relayPort);
		LED.set(Relay.Value.kForward);
		cam= CameraServer.getInstance().startAutomaticCapture(0);//create new camera instance
		cam.setResolution(320*2,240*2);//control pixel size
		visionT=new VisionThread(cam, new GripPipeline(), pipeline -> {
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
			
		});
		visionT.start();
	}

	/**
	 * This autonomous. You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		dtr.chassis.setSafetyEnabled(true);
		//This is default code we use to choose an autonomous
	}

	/**This function is called periodically during autonomous.**/
	@Override
	public void autonomousPeriodic() { 
		switch (m_autoSelected) {
			case kCustomAuto://testing autonomous
				autoSonar();
				break;
			case kDefaultAuto://nothing right now
				SmartDashboard.putNumber("centerX",centerX);
				LED.set(Relay.Value.kForward);
				break;
			default:
				// Put default auto code here
				break;
			
		}Timer.delay(.005);
	}
	
	//temporary autonomous to drive with sonar
	public void autoSonar() {
		dtr.chassis.setSafetyEnabled(true);//allow auto to happen
		dtr.getHeading();
		dtr.getDistance();
		dtr.getAccel();
		dtr.updateAxes();
			
		/*if (Math.round(currentDistance1)!=Math.round(currentDistance2)) {
		if (currentDistance1>currentDistance2) {
			rightMotor.set(.2);
		}else if (currentDistance1>currentDistance2) {
			leftMotor.set(-.2);
		}}*/
		//else {
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
	
	/**This function is called periodically during operator control.*/
	@Override
	public void teleopPeriodic() {
		dtr.chassis.setSafetyEnabled(true);
		dtr.changeDrive();
		dtr.updateAxes();
		LED.set(Relay.Value.kForward);
		getEnc();
		buttons();
		//teleopLift();
		
	}
	
	/**This function is called periodically during test mode**/
	@Override
	public void testPeriodic() {}
	
	public void buttons() {
		
		if(downButton == false && xbox.getRawButton(porting.butLBumper) == true)
		{
			if(positionCase != 0)
				positionCase--;
			directionLift = 0;
		}
		else if(upButton == false && xbox.getRawButton(porting.butRBumper) == true)
		{
			if(positionCase != 6)
				positionCase++;
			directionLift = 1;
		}
		SmartDashboard.putNumber("positionCase", positionCase);

		if(directionLift == 0 && limDown.get() == false)
		{
			switch(positionCase) {
			case 0: 
				liftLeft.set(0);
				break;
			case 1:
				if(enc1.getDistance() > 0)
					liftLeft.set(-0.5);
				else
					liftLeft.set(0);
				break;
			case 2:
				if(enc1.getDistance() > 20*4)
					liftLeft.set(-0.5);
				else
					liftLeft.set(0);
				break;
			case 3:
				if(enc1.getDistance() > 90*4)
					liftLeft.set(-0.5);
				else
					liftLeft.set(0);
				break;
			case 4:
				if(enc1.getDistance() > 110*4)
					liftLeft.set(-0.5);
				else
					liftLeft.set(0);
				break;
			}
		}
		if(directionLift == 1 && limUp.get() == false)
		{
			switch(positionCase) {
			case 2:
				if(enc1.getDistance() < 20*4)
					liftLeft.set(0.5);
				else
					liftLeft.set(0);
				break;
			case 3:
				if(enc1.getDistance() < 90*4)
					liftLeft.set(0.5);
				else
					liftLeft.set(0);
				break;
			case 4:
				if(enc1.getDistance() < 110*4)
					liftLeft.set(0.5);
				else
					liftLeft.set(0);
				break;
			case 5:
				if(enc1.getDistance() < 180*4)
					liftLeft.set(0.5);
				else
					liftLeft.set(0);
				break;
			case 6:
				liftLeft.set(0);
				break;
			}
		}
		
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
}
