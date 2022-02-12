package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
    private final WPI_TalonFX hangMotor = new WPI_TalonFX(Constants.MOTOR_HANG);
    private final WPI_VictorSPX climbMotor1 = new WPI_VictorSPX(Constants.MOTOR_CLIMB_1);
    private final WPI_VictorSPX climbMotor2 = new WPI_VictorSPX(Constants.MOTOR_CLIMB_2);

    public HangSubsystem() {
        // Factory reset motors
        climbMotor1.configFactoryDefault();
        climbMotor2.configFactoryDefault();
        hangMotor.configFactoryDefault();
        // Set motors' invert flag
        climbMotor1.setInverted(Constants.MOTOR_CLIMB_1_REVERSE);
        climbMotor2.setInverted(Constants.MOTOR_CLIMB_2_REVERSE);
        hangMotor.setInverted(Constants.MOTOR_HANG_REVERSE);
        // Set hang motor(Talon FX)'s current draw limit
        hangMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 38, 0.1));
        // Set motors' neutral mode to brake(prevent robot fall down when disable)
        climbMotor1.setNeutralMode(NeutralMode.Brake);
        climbMotor2.setNeutralMode(NeutralMode.Brake);
        hangMotor.setNeutralMode(NeutralMode.Brake);
        // Set motors' feedback sensor to integrated one
        climbMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        climbMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        hangMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        climbMotor1.setSensorPhase(Constants.ENCODER_CLIMB_1_REVERSE);
        climbMotor2.setSensorPhase(Constants.ENCODER_CLIMB_2_REVERSE);
    }

    @Override
    public void periodic() {

    }

    public void reset(){
        climbMotor1.setSelectedSensorPosition(0);
        climbMotor2.setSelectedSensorPosition(0);
        hangMotor.setSelectedSensorPosition(0);
    }

    public void riseHang(){
        hangMotor.set(ControlMode.Position, Constants.POSITION_HANG_EXTENDED);
    }

    public void collapseHang(){
        hangMotor.set(ControlMode.Position, 0);
    }

    public void riseClimber(){
        climbMotor1.set(ControlMode.Position, Constants.POSITION_CLIMB_EXTENDED);
        climbMotor2.set(ControlMode.Position, Constants.POSITION_CLIMB_EXTENDED);
    }

    public void collapseClimber(){
        climbMotor1.set(ControlMode.Position, 0);
        climbMotor2.set(ControlMode.Position, 0);
    }

    public void stop(){
        hangMotor.stopMotor();
        climbMotor1.stopMotor();
        climbMotor2.stopMotor();
    }
}

