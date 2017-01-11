package org.usfirst.frc.team78.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping.
 */
public class Robot extends IterativeRobot {
	private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.005; // proportional turning constant

	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
	private static final double kVoltsPerDegreePerSecond = 0.0128;

	private static final int kLeftMotorPort = 0;
	private static final int kRightMotorPort = 1;
	private static final int kGyroPort = 0;
	private static final int kJoystickPort = 0;

	private RobotDrive myRobot = new RobotDrive(kLeftMotorPort, kRightMotorPort);
	private AnalogGyro gyro = new AnalogGyro(kGyroPort);
	private Joystick joystick = new Joystick(kJoystickPort);

	@Override
	public void robotInit() {
		gyro.setSensitivity(kVoltsPerDegreePerSecond);
	}

	/**
	 * The motor speed is set from the joystick while the RobotDrive turning
	 * value is assigned from the error between the setpoint and the gyro angle.
	 */
	@Override
	public void teleopPeriodic() {
//		double turningValue = (kAngleSetpoint - gyro.getAngle()) * kP;
		// Invert the direction of the turn if we are going backwards
//		turningValue = Math.copySign(turningValue, joystick.getY());
//		myRobot.drive(joystick.getY(), turningValue);
		
/**
 * First try at heading correction for tank drive			
 */
		
		double turningValue = (kAngleSetpoint - gyro.getAngle()) * kP;
		double rightStick = joystick.getY();
		double leftStick = joystick.getThrottle();
		
		if (Math.abs(rightStick-leftStick) < .1)
		{
			double arcadeY = (leftStick+rightStick)/2;
			myRobot.drive(arcadeY, turningValue);
		}
		else
			myRobot.tankDrive(leftStick, rightStick);
				
		
	}
}
