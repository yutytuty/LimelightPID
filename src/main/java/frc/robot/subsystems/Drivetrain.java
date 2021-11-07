package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.command.drivetrains.commands.DriveTank;
import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends TankDrivetrain {

    private final static double WHEEL_DIAMETER = 6.0;
    private final static double METER_TO_INCH = 0.0254;
    private final static double PULSES_PER_ROTATION = 360.0;

    private double distancePerPulse = WHEEL_DIAMETER * METER_TO_INCH * Math.PI / PULSES_PER_ROTATION;
    private static Drivetrain instance;

    public static RootNamespace drivetrainNamespace = new RootNamespace("drivetrain");
    private static Namespace PID = drivetrainNamespace.addChild("PID");

    public static Supplier<Double> kP = PID.addConstantDouble("kP", 0);
    public static Supplier<Double> kI = PID.addConstantDouble("kI", 0);
    public static Supplier<Double> kD = PID.addConstantDouble("kD", 0);
    public static Supplier<Double> tolerance = PID.addConstantDouble("Tolerance", 0);
    public static Supplier<Double> waitTime = PID.addConstantDouble("Wait Time", 0);

    public static PIDSettings pidSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);

    private Encoder encoder;

    private Drivetrain() {
        super(new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN.DRIVETRAIN_LEFT_TALON),
                        new VictorSP(RobotMap.CAN.DRIVETRAIN_LEFT_VICTOR)),
                new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN.DRIVETRAIN_RIGHT_TALON),
                        new VictorSP(RobotMap.CAN.DRIVETRAIN_RIGHT_VICTOR)));
        encoder = new Encoder(2, 3);
        encoder.setDistancePerPulse(distancePerPulse);
    }

    @Override
    public void periodic() {
        drivetrainNamespace.update();
    }

    public Supplier<Double> getEncoderDistance() {
        return encoder::getDistance;
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    public void configureDashboard() {
        drivetrainNamespace.putData("move", new DriveTank(this, 0.6, 0.6));
        drivetrainNamespace.putData("move pid", new DriveTankWithPID(this, pidSettings, pidSettings,
                1.5, 1.5, encoder::getDistance, encoder::getDistance));
    }
}
