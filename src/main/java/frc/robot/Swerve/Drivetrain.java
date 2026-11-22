package frc.robot.Swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotMap;

public class Drivetrain extends SwerveDrivetrain {

    private static final String NAMESPACE_NAME = "drivetrain";
    private static final CANBus canivore = new CANBus("canivore");

    private static final double TRACK_WIDTH = 0.55;
    private static final double TRACK_LENGTH = 0.55;
    private static final double MAX_VELOCITY = 5.1;
    private static final int GYRO_OFFSET = 180;

    private final Pigeon2 gyro;
    private static Drivetrain instance;

    private Drivetrain(String NAMESPACE_NAME, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
                       SwerveModule backRight, double trackWidth, double tracklength, double maxVelocity, Pigeon2 gyro){
        super(NAMESPACE_NAME, frontLeft, frontRight, backLeft, backRight, trackWidth, tracklength, maxVelocity);
        this.gyro = gyro;
        configureDashboard();
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID, canivore));
        }
        return instance;
    }


    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees());
    }

    @Override
    public void resetAngleSensor() {
        gyro.setYaw(GYRO_OFFSET);
    }

    @Override
    public void periodic() {
        super.periodic();
        namespace.update();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro", () -> getAngle().getDegrees());
    }
}
