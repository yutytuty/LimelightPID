package frc.robot.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;


/**
 * A command that moves a drivetrain using PID and Limelight.
 *
 * @see DriveTankWithPID
 */
public class DriveTankWithLimelightPID extends CommandBase {

    /**
     * The starting distance of the limelight from the target in meters.
     */
    private double startingDistance;

    /**
     * The command this command is using to operate.
     */
    private DriveTankWithPID command;

    /**
     * The limelight this command uses.
     */
    private Limelight limelight;

    /**
     * The drivetrain this command operates on.
     */
    private TankDrivetrain drivetrain;

    /**
     * The PID Settings for the PID loop operating on the left side of the drivetrain.
     */
    private PIDSettings leftPIDSettings;

    /**
     * The PID Settings for the PID loop operating on the right side of the drivetrain.
     */
    private PIDSettings rightPIDSettings;

    /**
     * The setpoint the left side of the drivetrain should reach.
     */
    private Supplier<Double> leftSetpoint;

    /**
     * The setpoint the right side of the drivetrain should reach.
     */
    private Supplier<Double> rightSetpoint;

    /**
     * The error of the left side of the drivetrain from the target.
     */
    private Supplier<Double> leftError;

    /**
     * The error of the right side of the drivetrain from the target.
     */
    private Supplier<Double> rightError;

    public DriveTankWithLimelightPID(Limelight limelight, TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                                     Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftError,
                                     Supplier<Double> rightError) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.leftPIDSettings = leftPIDSettings;
        this.rightPIDSettings = rightPIDSettings;
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftError = leftError;
        this.rightError = rightError;
    }

    @Override
    public void initialize() {
        startingDistance = limelight.calculateDistance();
        command = new DriveTankWithPID(drivetrain, leftPIDSettings, rightPIDSettings, leftSetpoint, rightSetpoint,
                () -> startingDistance - leftError.get(), () -> startingDistance - rightError.get());
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
