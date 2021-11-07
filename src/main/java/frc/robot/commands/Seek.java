package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

/**
 * A command that revolves the robot until the Limelight detects a valid target.
 */
public class Seek extends CommandBase {

    private DriveArcade command;
    private Limelight limelight;

    public Seek(Limelight limelight, double speed, int pipeline) {
        this.limelight = limelight;
        limelight.setPipeline(pipeline);
        this.command = new DriveArcade(Drivetrain.getInstance(), 0.0, speed);
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return limelight.isOnTarget();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
