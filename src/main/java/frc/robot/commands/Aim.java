package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.OrientWithPID;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;

/**
 * A command that aims the robot on the target using Limelight.
 */
public class Aim extends CommandBase {

    private static RootNamespace aimNamespace = new RootNamespace("aim");
    public static Namespace PID = aimNamespace.addChild("PID");

    public static Supplier<Double> kP = PID.addConstantDouble("kP", 0);
    public static Supplier<Double> kI = PID.addConstantDouble("kI", 0);
    public static Supplier<Double> kD = PID.addConstantDouble("kD", 0);
    public static Supplier<Double> tolerance = PID.addConstantDouble("Tolerance", 0);
    public static Supplier<Double> waitTime = PID.addConstantDouble("Wait Time", 0);
    public static Supplier<Double> kS = PID.addConstantDouble("kS", 0);

    public static PIDSettings pidSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);
    public static FeedForwardSettings feedForwardSettings = new FeedForwardSettings(kS, () -> 0.0, () -> 0.0);

    private OrientWithPID command;

    public Aim(Limelight limelight) {
        this.command = new OrientWithPID(Drivetrain.getInstance(), () -> -limelight.getHorizontalOffsetFromTarget(),
                0.0, pidSettings, feedForwardSettings);
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
