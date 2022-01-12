package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    public boolean inverted = false;

    private double lastSpeed = 0;

    private final CANSparkMax leftMotor_1 = new CANSparkMax(Constants.MOTOR_CHANNEL_LEFT_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftMotor_2 = new CANSparkMax(Constants.MOTOR_CHANNEL_LEFT_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor_1 = new CANSparkMax(Constants.MOTOR_CHANNEL_RIGHT_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor_2 = new CANSparkMax(Constants.MOTOR_CHANNEL_RIGHT_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    // The motors on the left side of the drive.
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor_1, leftMotor_2);

    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor_1, rightMotor_2);

    // The robot's drive
    private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    private DifferentialDriveOdometry odometry;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        initializeEncoders();
        odometry = new DifferentialDriveOdometry(navX.getRotation2d());
    }

    public void setSafetyEnabled(boolean enabled){
        drive.setSafetyEnabled(enabled);
    }

    @Override
    public void periodic() {
        if(lastSpeed!=0){
            drive.arcadeDrive(lastSpeed,0);
        }
        // Update the odometry in the periodic block
        odometry.update(navX.getRotation2d(), m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        initializeEncoders();
        odometry.resetPosition(pose, navX.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
        lastSpeed=fwd;
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(inverted?-rightVolts:leftVolts);
        rightMotors.setVoltage(inverted?leftVolts:-rightVolts);
        drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void initializeEncoders() {
        leftMotor_1.getEncoder().setPosition(0);
        leftMotor_2.getEncoder().setPosition(0);
        rightMotor_1.getEncoder().setPosition(0);
        rightMotor_2.getEncoder().setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        navX.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -navX.getRate();
    }

    public void reverse(){
        inverted=!inverted;
        m_leftEncoder.close();
        m_rightEncoder.close();
        drive.close();
        if(inverted){
            drive = new DifferentialDrive(rightMotors, leftMotors);
            m_leftEncoder = new Encoder(DriveConstants.kEncoder_R_Channel1, DriveConstants.kEncoder_R_Channel2,
                    !DriveConstants.kEncoder_R_Reversed, CounterBase.EncodingType.k1X);
            m_rightEncoder = new Encoder(DriveConstants.kEncoder_L_Channel1, DriveConstants.kEncoder_L_Channel2,
                    !DriveConstants.kEncoder_L_Reversed, CounterBase.EncodingType.k1X);
        }else{
            drive = new DifferentialDrive(leftMotors, rightMotors);
            m_leftEncoder = new Encoder(DriveConstants.kEncoder_L_Channel1, DriveConstants.kEncoder_L_Channel2,
                    DriveConstants.kEncoder_L_Reversed, CounterBase.EncodingType.k1X);
            m_rightEncoder = new Encoder(DriveConstants.kEncoder_R_Channel1, DriveConstants.kEncoder_R_Channel2,
                    DriveConstants.kEncoder_R_Reversed, CounterBase.EncodingType.k1X);
        }
        initializeEncoders();
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoder_DistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoder_DistancePerPulse);
    }
}