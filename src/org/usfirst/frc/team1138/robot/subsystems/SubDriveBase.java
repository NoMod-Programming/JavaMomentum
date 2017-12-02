package org.usfirst.frc.team1138.robot.subsystems;

import com.ctre.PigeonImu;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1138.robot.RobotMap;
import org.usfirst.frc.team1138.robot.commands.DriveWithJoy;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * @author Zheyuan Hu
 * @version 1.0.0
 */
public class SubDriveBase extends Subsystem{
	private CANTalon leftFrontBaseMotor, rightFrontBaseMotor, leftRearBaseMotor, rightRearBaseMotor, gyroTalon;
	private PigeonImu pigeonImu;
	private DoubleSolenoid shifterSolenoid, liftSolenoid;
	//private AHRS gyroAccel; deprecated

    public SubDriveBase() {
		// Motors
		// master motors
		leftFrontBaseMotor = new CANTalon(RobotMap.KLeftFrontBaseTalon);
		rightFrontBaseMotor = new CANTalon(RobotMap.KRightFrontBaseTalon);
		//slave motors 
		leftRearBaseMotor = new CANTalon(RobotMap.KLeftRearBaseTalon);
		rightRearBaseMotor = new CANTalon(RobotMap.KRightRearBaseTalon);
		// Config the masters 
		leftFrontBaseMotor.setInverted(true);
		initSafeMotor();
		leftFrontBaseMotor.enableControl();
		rightFrontBaseMotor.enableControl();
		// Config the slaves
		leftRearBaseMotor.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightRearBaseMotor.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftRearBaseMotor.set(leftFrontBaseMotor.getDeviceID());
		rightRearBaseMotor.set(rightFrontBaseMotor.getDeviceID());
		
		// Solenoids 
		shifterSolenoid = new DoubleSolenoid(RobotMap.KShifterSolenoid1, RobotMap.KShifterSolenoid2);
		liftSolenoid = new DoubleSolenoid(RobotMap.KLiftSolenoid1, RobotMap.KLiftSolenoid2); 
		
		//Gyro & Accel
		//gyroAccel = new AHRS(Port.kMXP);
        gyroTalon = new CANTalon(3);
         pigeonImu = new PigeonImu(gyroTalon);
		SmartDashboard.putNumber("Navx Connection: ", pigeonImu.GetFirmVers());
		pigeonImu.SetYaw(0);
		//gyroAccel.zeroYaw();
		
		//Encoders 
		leftFrontBaseMotor.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		rightFrontBaseMotor.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		leftFrontBaseMotor.configEncoderCodesPerRev(4095);
		rightFrontBaseMotor.configEncoderCodesPerRev(4095);
		leftFrontBaseMotor.setEncPosition(0);
		rightFrontBaseMotor.setEncPosition(0);
		
		// LiveWindow
//       LiveWindow.addSensor("SubDriveBase", "Pigeon", (LiveWindowSendable) pigeonImu);
//		LiveWindow.addSensor("SubDriveBase", "Gyro", gyroAccel);
//		LiveWindow.addActuator("SubDriveBase", "Left Front Motor", leftFrontBaseMotor);
//		LiveWindow.addActuator("SubDriveBase", "Right Front Motor", rightFrontBaseMotor);
//		LiveWindow.addActuator("SubDriveBase", "Left Rear Motor", leftRearBaseMotor);
//		LiveWindow.addActuator("SubDriveBase", "Right Rear Motor", rightRearBaseMotor);
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoy());
	}


    /**
     * Enable Safety for motors
     */
    private void initSafeMotor() {
		leftFrontBaseMotor.setSafetyEnabled(true);
		rightFrontBaseMotor.setSafetyEnabled(true);
		leftRearBaseMotor.setSafetyEnabled(true);
		rightRearBaseMotor.setSafetyEnabled(true);
	}

    /**
     * @apiNote The side with the GEAR IS THE FRONT!
     * @param left  the motor input for left motors from -1.0 to 1.0
     * @param right the motor input for right motors from -1.0 to 1.0
     */
    public void tankDrive(double left, double right) {
		if(left > RobotMap.KDeadZoneLimit || left < -RobotMap.KDeadZoneLimit) leftFrontBaseMotor.set(left);
		else leftFrontBaseMotor.set(0);
		if(right > RobotMap.KDeadZoneLimit || right < -RobotMap.KDeadZoneLimit) rightFrontBaseMotor.set(right);
		else rightFrontBaseMotor.set(0);
	}

    public void drive(double right, double curve) { //TODO fix here!
        final double leftOutput;
        final double rightOutput;
        double m_sensitivity = 1.0;
        if (curve < 0) {
            double value = Math.log(-curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = right / ratio;
            rightOutput = right;
            System.out.println("Turn Left: " + "lout: " + leftOutput + "rout: " + rightOutput);
        } else if (curve > 0) {
            double value = Math.log(curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = right;
            rightOutput = right / ratio;
            System.out.println("Turn Right: " + "lout: " + leftOutput + "rout: " + rightOutput);
        } else {
            leftOutput = right;
            rightOutput = right;
            System.out.println("Straight: " + "lout: " + leftOutput + "rout: " + rightOutput);
        }
        setLeftRightMotorOutputs(leftOutput, rightOutput);
    }

    /**
     * Set the speed of the right and left motors. This is used once an appropriate drive setup
     * function is called such as twoWheelDrive(). The motors are set to "leftSpeed" and
     * "rightSpeed" and includes flipping the direction of one side for opposing motors.
     *
     * @param leftOutput  The speed to send to the left side of the robot.
     * @param rightOutput The speed to send to the right side of the robot.
     */
    private void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        leftFrontBaseMotor.set(limit(leftOutput));
        leftRearBaseMotor.set(limit(leftOutput));
        rightFrontBaseMotor.set(-limit(rightOutput));
        rightRearBaseMotor.set(-limit(rightOutput));
        System.out.println("lmotor: " + limit(leftOutput) + "rmotor: " + (-limit(rightOutput)));
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    private static double limit(double number) {
        if (number > 1.0) {
            return 1.0;
        }
        if (number < -1.0) {
            return -1.0;
        }
        return number;
    }

    /**
     * Shift the base to high position
     */
    private void highShiftBase() {
		shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

    /**
     * Shift the base to low position
     */
    private void lowShiftBase() {
		shifterSolenoid.set(DoubleSolenoid.Value.kForward);
	}

    /**
     * public method to switch shifts base
     */
    public void toggleShift() {
		if (shifterSolenoid.get() == DoubleSolenoid.Value.kForward) {
			highShiftBase();
		} else {
			lowShiftBase();
		}
	}

    /**
     * public method to engage the lift
     */
    public void engageLift()
	{
		if(liftSolenoid.get() == DoubleSolenoid.Value.kForward) //is the lift engaged?
			liftSolenoid.set((DoubleSolenoid.Value.kReverse));	//disengage lift
		else liftSolenoid.set((DoubleSolenoid.Value.kForward));	//engage lift
	}

    /**
     * public method to reset Gyro value
     */
    public void resetGyro() {
        pigeonImu.SetYaw(0);
		//gyroAccel.zeroYaw();
	}

    /**
     * @return Current Gyro Value in degrees from 180.0 to -180.0
     */
    public double getAngle() {
        double[] ypr = new double[3];
        pigeonImu.GetYawPitchRoll(ypr);
        return (-ypr[0]);
//		return gyroAccel.getAngle();
	}

    /**
     * @return the state of the lift solenoid
     */
    public Value getLiftState() {
		return liftSolenoid.get(); 
	}
}
