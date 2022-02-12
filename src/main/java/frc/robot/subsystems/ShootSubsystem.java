package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
    private final WPI_TalonFX shootMotor = new WPI_TalonFX(Constants.MOTOR_SHOOT);
    private final WPI_VictorSPX rotateMotor = new WPI_VictorSPX(Constants.MOTOR_ROTATE);
    private final CANSparkMax angleMotor = new CANSparkMax(Constants.MOTOR_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final PIDController pid_Rotation = new PIDController(Constants.PID_ROTATE[0], Constants.PID_ROTATE[1], Constants.PID_ROTATE[2]);
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private boolean autoAlignment = true;
    public ShootSubsystem() {
        shootMotor.configFactoryDefault();
        rotateMotor.configFactoryDefault();
        angleMotor.restoreFactoryDefaults();

        rotateMotor.setInverted(Constants.MOTOR_ROTATE_INVERTED);
        angleMotor.setInverted(Constants.MOTOR_ANGLE_INVERTED);

        shootMotor.setNeutralMode(NeutralMode.Coast);
        rotateMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if (autoAlignment){
            boolean hasTarget=limelight.getEntry("tv").getDouble(0)>=1.0;
            double currentRotationOutput=pid_Rotation.calculate(hasTarget?limelight.getEntry("tx").getDouble(-Constants.ALIGNMENT_X_OFFSET):-Constants.ALIGNMENT_X_OFFSET);
            rotateMotor.set(currentRotationOutput);
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

