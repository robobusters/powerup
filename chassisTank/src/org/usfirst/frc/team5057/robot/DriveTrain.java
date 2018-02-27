package org.usfirst.frc.team5057.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
	double leftY;
	double leftX;
	double rightY;
	double rightX;
	double dampen;
	boolean dpUp;
	boolean dpDown;
	boolean dpLeft;
	boolean dpRight;
	int mode;
	
	DifferentialDrive chassis;
	XboxController xbox;
	JoystickLocations porting;
	
	private static final int kUltrasonicPort1 = 0;
	private AnalogInput m_ultrasonic1 = new AnalogInput(kUltrasonicPort1);
	private static final int kUltrasonicPort2 = 2;
	private AnalogInput m_ultrasonic2 = new AnalogInput(kUltrasonicPort2);
	private static final double kValueToInches = 0.049;
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	Accelerometer accel= new BuiltInAccelerometer(Accelerometer.Range.k4G);
	
	double currentDistance1;
	double currentDistance2;
	double[] accVal = new double[3];
	
	public DriveTrain(DifferentialDrive d,XboxController x, JoystickLocations p) {
		chassis=d;
		xbox=x;
		porting=p;
	}
	
	public void updateAxes() {
		leftY=xbox.getRawAxis(porting.lYAxis);
		leftX=xbox.getRawAxis(porting.lXAxis);
		rightX=xbox.getRawAxis(porting.rXAxis);
		rightY=xbox.getRawAxis(porting.rYAxis);
		dampen=1-3/4*xbox.getTriggerAxis(Hand.kRight);
		SmartDashboard.putNumber("leftY", leftY);
		SmartDashboard.putNumber("leftX", leftX);
		SmartDashboard.putNumber("rightY", rightY);
		SmartDashboard.putNumber("rightX", rightX);
		SmartDashboard.putNumber("dampen", dampen);
		
		if(Math.round(xbox.getPOV())!=1) {
			if(Math.round(xbox.getPOV())<90) {
				dpUp = true;
				dpDown = false;
				dpLeft = false;
				dpRight = false;
			}else if(Math.round(xbox.getPOV())<180) {
				dpUp = false;
				dpDown = false;
				dpLeft = false;
				dpRight = true;
			}else if(Math.round(xbox.getPOV())<270) {
				dpUp = false;
				dpDown = true;
				dpLeft = false;
				dpRight = false;
			}else if(Math.round(xbox.getPOV())<360) {
				dpUp = false;
				dpDown = false;
				dpLeft = true;
				dpRight = false;
			}
		}else {
			
				dpUp = false;
				dpDown = false;
				dpLeft = false;
				dpRight = false;
			
		}
	}
	

	//This changes which kind of drive system to use. Ignore for now
	public void changeDrive() {
		if(xbox.getAButton())mode=1;
		else if(xbox.getBButton())mode=2;
		else if(xbox.getXButton())mode=3;
		else if(xbox.getYButton())mode=4;
		
		switch (mode) {
		case 1://button A does field oriented arcade
			driveArcade();
			break;
		case 2://button B does just arcade
			chassis.arcadeDrive(-xbox.getRawAxis(porting.lYAxis),-xbox.getRawAxis(porting.lXAxis));
			getHeading();
			getDistance();
			getAccel();
			break;
		case 3://button X does tank drive
			getHeading();
			getDistance();
			getAccel();
			chassis.tankDrive(-xbox.getRawAxis(porting.rYAxis),-xbox.getRawAxis(porting.lYAxis) );
			break;
		case 4://button Y drive this like a car
			chassis.curvatureDrive(-xbox.getRawAxis(porting.lYAxis),-xbox.getRawAxis(porting.rXAxis) , false);//cheeeeessssyyyy
			getHeading();
			getDistance();
			getAccel();
			break;
		default: break;
		}
	}
	
	//drive with arcade drive with respect to the field. Left joystick only
	public void driveArcade() {		
		double radians = getHeading()*Math.PI/180;//convert angle to radian value
		
		getHeading();//update gyro
		getDistance();//update sonar
		getAccel();//update accelerometer
		
		//math to find how to turn robot to make field oriented work
		double temp = Math.cos(radians)*leftX + Math.sin(radians)*leftY;
		leftY = -leftX*Math.sin(radians) + leftY*Math.cos(radians);
		leftX=temp;
						
		chassis.arcadeDrive( -this.dampen*leftY, -this.dampen*leftX);//drive
		
		if(Math.abs(xbox.getRawAxis(porting.rXAxis)) > .25) {
			chassis.arcadeDrive(0, -xbox.getRawAxis(porting.rXAxis));
		}
		
	}
	
	//updates the value of the sonar sensor
	public void getDistance() {
		this.currentDistance1 = m_ultrasonic1.getValue()*this.kValueToInches;
		SmartDashboard.putNumber("distance right", currentDistance1);
		
		this.currentDistance2 = m_ultrasonic2.getValue()*this.kValueToInches;
		SmartDashboard.putNumber("distance left", currentDistance2);
	}
	

	//updates the value of the gyro
	public double getHeading() {
		SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
		return gyro.getAngle();
	}
	
	//updates the value of the accelerometer Ignore for now
	public double[] getAccel() {
		accVal[0] = accel.getX()*9.81;
		accVal[1] = accel.getY()*9.81;
		accVal[2] = accel.getZ()*9.81;
		SmartDashboard.putNumber("x accel", accVal[0]);
		SmartDashboard.putNumber("y accel", accVal[1]);
		SmartDashboard.putNumber("z accel", accVal[2]);
		return accVal;
	}
	
	public boolean turnRight(double degrees) {
		if(degrees>gyro.getAngle()) {
			chassis.tankDrive(-.5, .5);
			return false;
		}
		chassis.arcadeDrive(0, 0);
		return true;
	}
	
	public boolean turnLeft(double degrees) {
		if(-degrees<gyro.getAngle()) {
			chassis.tankDrive(.5, -.5);
			return false;
		}
		chassis.arcadeDrive(0, 0);
		return true;
	}
	
	public boolean driveUntil(double inches) {
		getDistance();
		if(currentDistance1==inches||currentDistance1==inches) {
			chassis.arcadeDrive(0, 0);
			return true;
		}
		chassis.arcadeDrive(.625, 0);
		return false;
	}
}
