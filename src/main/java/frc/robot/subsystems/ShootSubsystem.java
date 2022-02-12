package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
    private final WPI_TalonFX shootMotor = new WPI_TalonFX(Constants.MOTOR_SHOOT);
    private final WPI_VictorSPX rotateMotor = new WPI_VictorSPX(Constants.MOTOR_ROTATE);
    private final CANSparkMax angleMotor = new CANSparkMax(Constants.MOTOR_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private boolean autoAlignment = true;
    public ShootSubsystem() {
        shootMotor.configFactoryDefault();
        rotateMotor.configFactoryDefault();
        angleMotor.restoreFactoryDefaults();

        shootMotor.setNeutralMode(NeutralMode.Coast);
        rotateMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if (autoAlignment){
            //TODO implement the auto alignment algorithm
        }
    }

    public void enableShootMotor(){
        shootMotor.set(ControlMode.Velocity, Constants.SHOOT_SPEED*Constants.FALCON_MAX_RPM);
    }
    public void disableShootMotor(){
        shootMotor.stopMotor();
    }
    public void enableAutoAlignment(){
        autoAlignment=true;
    }
    public void disableAutoAlignment(){
        autoAlignment=false;
    }
    public boolean isAutoAlignmentEnabled(){
        return autoAlignment;
    }
}

