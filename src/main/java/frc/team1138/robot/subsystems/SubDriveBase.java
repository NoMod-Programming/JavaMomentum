package frc.team1138.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1138.robot.RobotMap;
import frc.team1138.robot.commands.DriveWithJoy;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix. motorcontrol.can.TalonSRX;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * @author Zheyuan Hu
 * @version 1.0.0
 */
public class SubDriveBase extends Subsystem{
	private TalonSRX leftFrontBaseMotor, rightFrontBaseMotor, leftRearBaseMotor, rightRearBaseMotor, gyroTalon;
	private PigeonIMU PigeonIMU;
	private DoubleSolenoid shifterSolenoid, liftSolenoid;
	//private AHRS gyroAccel; deprecated

    public SubDriveBase() {
		// Motors
		// master motors
		leftFrontBaseMotor = new TalonSRX(RobotMap.KLeftFrontBaseTalon);
		rightFrontBaseMotor = new TalonSRX(RobotMap.KRightFrontBaseTalon);
		//slave motors 
		leftRearBaseMotor = new TalonSRX(RobotMap.KLeftRearBaseTalon);
		rightRearBaseMotor = new TalonSRX(RobotMap.KRightRearBaseTalon);
		// Config the masters 
		leftFrontBaseMotor.setInverted(true);
//		initSafeMotor();
		// Config the slaves
		leftRearBaseMotor.set(ControlMode.Follower,leftFrontBaseMotor.getDeviceID());
		rightRearBaseMotor.set(ControlMode.Follower,rightFrontBaseMotor.getDeviceID());
		
		// Solenoids 
		shifterSolenoid = new DoubleSolenoid(RobotMap.KShifterSolenoid1, RobotMap.KShifterSolenoid2);
		liftSolenoid = new DoubleSolenoid(RobotMap.KLiftSolenoid1, RobotMap.KLiftSolenoid2); 
		
		//Gyro & Accel
		//gyroAccel = new AHRS(Port.kMXP);	
        gyroTalon = new TalonSRX(3);
        PigeonIMU = new PigeonIMU(gyroTalon);
		SmartDashboard.putNumber("Navx Connection: ", PigeonIMU.getFirmwareVersion());
		PigeonIMU.setYaw(0,0);
		//gyroAccel.zeroYaw();
		
		//Encoders 
		leftFrontBaseMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
		rightFrontBaseMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
//		leftFrontBaseMotor.configEncoderCodesPerRev (4095);
//		rightFrontBaseMotor.configEncoderCodesPerRev(4095);
		leftFrontBaseMotor.getSensorCollection().setQuadraturePosition(0,0);
		rightFrontBaseMotor.getSensorCollection().setQuadraturePosition(0,0);
		
		// LiveWindow
//       LiveWindow.addSensor("SubDriveBase", "Pigeon", (LiveWindowSendable) PigeonIMU);
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
//    private void initSafeMotor() {
//		leftFrontBaseMotor.setSafetyEnabled(true);
//		rightFrontBaseMotor.setSafetyEnabled(true);
//		leftRearBaseMotor.setSafetyEnabled(true);
//		rightRearBaseMotor.setSafetyEnabled(true);
//	}

    /**
     * @apiNote The side with the GEAR IS THE FRONT!
     * @param left  the motor input for left motors from -1.0 to 1.0
     * @param right the motor input for right motors from -1.0 to 1.0
     */
    public void tankDrive(double left, double right) {
    	
		if(left > RobotMap.KDeadZoneLimit || left < -RobotMap.KDeadZoneLimit) 
		{
			leftFrontBaseMotor.set(ControlMode.PercentOutput,left);
			
			SmartDashboard.putNumber("Tank Drive", leftFrontBaseMotor.getMotorOutputVoltage()); 
		}
		else
		{
			leftFrontBaseMotor.set(ControlMode.PercentOutput,0);
			SmartDashboard.getString("Not working","No");
		}
		
		
		if(right > RobotMap.KDeadZoneLimit || right < -RobotMap.KDeadZoneLimit) 
		{
			rightFrontBaseMotor.set(ControlMode.PercentOutput,right);
			SmartDashboard.putNumber("Tank Drive", rightFrontBaseMotor.getMotorOutputVoltage()); 
		}
		else
		{
			rightFrontBaseMotor.set(ControlMode.PercentOutput,0);
			SmartDashboard.getString("Not working","No");
		}
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
        leftFrontBaseMotor.set(ControlMode.PercentOutput,limit(leftOutput));
        leftRearBaseMotor.set(ControlMode.PercentOutput,limit(leftOutput));
        rightFrontBaseMotor.set(ControlMode.PercentOutput,-limit(rightOutput));
        rightRearBaseMotor.set(ControlMode.PercentOutput,-limit(rightOutput));
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
        PigeonIMU.setYaw(0,0);
		//gyroAccel.zeroYaw();
	}
    
    public void DriveForward(float distance, float speed) {
    	float encoderReference = leftFrontBaseMotor.getSensorCollection().getQuadraturePosition(); //Alex Harris fixed it
    	float encoderReference2 = rightFrontBaseMotor.getSensorCollection().getQuadraturePosition(); 
    	float encoder = leftFrontBaseMotor.getSensorCollection().getQuadraturePosition(); 
    	float encoder2= rightFrontBaseMotor.getSensorCollection().getQuadraturePosition(); 
    	while((encoder - encoderReference) < distance && (encoder2 - encoderReference2) < distance)
    	{
    		rightFrontBaseMotor.set(ControlMode.PercentOutput, speed);
    		leftFrontBaseMotor.set(ControlMode.PercentOutput, speed);
    		encoder = leftFrontBaseMotor.getSensorCollection().getQuadraturePosition();
    		encoder2 = rightFrontBaseMotor.getSensorCollection().getQuadraturePosition();
    	}
    }
    /**
     * @return Current Gyro Value in degrees from 180.0 to -180.0
     */
    public double getAngle() {
        double[] ypr = new double[3];
        PigeonIMU.getYawPitchRoll(ypr);
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
