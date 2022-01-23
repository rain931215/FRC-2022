package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;


public class RunTrajectoryCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private SequentialCommandGroup command;
    private Trajectory trajectory;

    public RunTrajectoryCommand(DriveSubsystem driveSubsystem, String pathName) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            // Run path following command, then stop at the end.
            command = new RamseteCommand(
                    trajectory,
                    driveSubsystem::getPose,
                    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                    new SimpleMotorFeedforward(Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    driveSubsystem.getDriveKinematics(),
                    driveSubsystem::getWheelSpeeds,
                    new PIDController(Constants.kPDriveVel, 0, 0),
                    new PIDController(Constants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    driveSubsystem::tankDriveVolts,
                    driveSubsystem
            ).andThen(new InstantCommand(() -> {
                driveSubsystem.tankDriveVolts(0, 0);
            }));
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
        }
    }

    @Override
    public void initialize() {
        if (trajectory == null) return;
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
        command.initialize();
    }

    @Override
    public void execute() {
        if (trajectory == null) return;
        command.execute();
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) return true;
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (trajectory == null) return;
        command.end(interrupted);
    }
}
