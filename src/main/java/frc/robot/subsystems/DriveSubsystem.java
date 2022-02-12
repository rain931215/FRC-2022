package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    // invert flag
    public boolean inverted = false;

    // Spark Max motor controllers
    private final CANSparkMax leftMotor_1 = new CANSparkMax(Constants.MOTOR_CHASSIS_LEFT_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftMotor_2 = new CANSparkMax(Constants.MOTOR_CHASSIS_LEFT_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor_1 = new CANSparkMax(Constants.MOTOR_CHASSIS_RIGHT_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor_2 = new CANSparkMax(Constants.MOTOR_CHASSIS_RIGHT_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AHRS navX = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);

    // The robot's drive
    private DifferentialDrive drive = new DifferentialDrive(leftMotor_1, rightMotor_1);

    private final DifferentialDriveOdometry odometry;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        leftMotor_1.restoreFactoryDefaults();
        leftMotor_2.restoreFactoryDefaults();
        leftMotor_1.setInverted(Constants.LEFT_INVERTED);
        leftMotor_2.follow(leftMotor_1);
        rightMotor_1.restoreFactoryDefaults();
        rightMotor_2.restoreFactoryDefaults();
        rightMotor_1.setInverted(Constants.RIGHT_INVERTED);
        rightMotor_2.follow(rightMotor_2);
        odometry = new DifferentialDriveOdometry(navX.getRotation2d());
        initializeEncoders();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(navX.getRotation2d(), getLeftEncoder().getPosition(),
                getRightEncoder().getPosition());
    }

    public void setSafetyEnabled(boolean enabled) {
        drive.setSafetyEnabled(enabled);
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
        if (inverted) {
            return new DifferentialDriveWheelSpeeds(rightMotor_1.getEncoder().getVelocity(), leftMotor_1.getEncoder().getVelocity());
        }
        return new DifferentialDriveWheelSpeeds(leftMotor_1.getEncoder().getVelocity(), rightMotor_1.getEncoder().getVelocity());
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
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor_1.setVoltage(inverted ? -rightVolts : leftVolts);
        rightMotor_1.setVoltage(inverted ? leftVolts : -rightVolts);
        drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void initializeEncoders() {
        getLeftEncoder().setPosition(0);
        getRightEncoder().setPosition(0);
        getLeftEncoder().setPositionConversionFactor(Constants.ENCODER_SCALE_FACTOR_POSITION);
        getRightEncoder().setPositionConversionFactor(Constants.ENCODER_SCALE_FACTOR_POSITION);
        getLeftEncoder().setVelocityConversionFactor(Constants.ENCODER_SCALE_FACTOR_VELOCITY);
        getRightEncoder().setVelocityConversionFactor(Constants.ENCODER_SCALE_FACTOR_VELOCITY);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (getLeftEncoder().getPosition() + getRightEncoder().getPosition()) / 2.0;
    }

    public CANSparkMax getLeftMotor() {
        if (inverted) {
            return rightMotor_1;
        } else {
            return leftMotor_1;
        }
    }

    public CANSparkMax getRightMotor() {
        if (inverted) {
            return leftMotor_1;
        } else {
            return rightMotor_1;
        }
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public RelativeEncoder getLeftEncoder() {
        return getLeftMotor().getEncoder();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public RelativeEncoder getRightEncoder() {
        return getRightMotor().getEncoder();
    }

    public DifferentialDriveKinematics getDriveKinematics() {
        return driveKinematics;
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

    public void reverse() {
        setInverted(!getInverted());
    }

    public void setInverted(boolean _inverted) {
        inverted = _inverted;
        drive.close(); // Close DifferentialDrive object
        if (inverted) {
            leftMotor_1.setInverted(!Constants.LEFT_INVERTED);
            rightMotor_1.setInverted(!Constants.RIGHT_INVERTED);
            drive = new DifferentialDrive(rightMotor_1, leftMotor_1);
        } else {
            leftMotor_1.setInverted(Constants.LEFT_INVERTED);
            rightMotor_1.setInverted(Constants.RIGHT_INVERTED);
            drive = new DifferentialDrive(leftMotor_1, rightMotor_1);
        }
        leftMotor_2.follow(leftMotor_1);
        rightMotor_2.follow(rightMotor_1);
        initializeEncoders();
    }

    public boolean getInverted() {
        return inverted;
    }
}