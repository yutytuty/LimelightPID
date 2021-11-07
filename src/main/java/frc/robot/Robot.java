// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.drivetrains.commands.DriveTank;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim;
import frc.robot.commands.DriveTankWithLimelightPID;
import frc.robot.commands.Seek;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private final RootNamespace rootNamespace = new RootNamespace("root namespace");
    private final Namespace namespace = rootNamespace.addChild("namespace");
    private Command m_autonomousCommand;
    public static OI oi;
    private Limelight limelight;
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
//        Encoder leftEncoder = new Encoder(0, 1);
//        Encoder rightEncoder = new Encoder(2, 3);
//        namespace.putNumber("left encoder", () -> leftEncoder.getDistance());
//        namespace.putNumber("right encoder", () -> rightEncoder.getDistance());
        oi = new OI();
        limelight = new Limelight();
        Drivetrain.drivetrainNamespace.update();
        namespace.putData("Seek single", new Seek(limelight, 0.45, 0));
        namespace.putData("Seek double", new SequentialCommandGroup(new Seek(limelight, 0.45, 0),
                new Seek(limelight, 0.2, 1)));
        namespace.putData("Aim", new Aim(limelight));
        namespace.putData("Drive to target", new DriveTankWithLimelightPID(limelight, drivetrain,
                Drivetrain.pidSettings, Drivetrain.pidSettings, () -> 0.0 + 0.5,
                () -> 0.0 + 0.5, () -> limelight.calculateDistance(),
                () -> limelight.calculateDistance()));
        namespace.putData("Drive forward", new DriveTank(drivetrain, () -> 0.45, () -> 0.45,
                () -> !limelight.isOnTarget()));
        namespace.putData("Seek and aim single", new SequentialCommandGroup(new Seek(limelight, 0.45, 0),
                new Aim(limelight)));
        namespace.putData("Seek and aim double", new SequentialCommandGroup(new Seek(limelight, 0.45, 0),
                new Seek(limelight, 0.2, 1), new Aim(limelight)));

        limelight.setPipeline(0);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        rootNamespace.update();
        limelight.periodic();
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
