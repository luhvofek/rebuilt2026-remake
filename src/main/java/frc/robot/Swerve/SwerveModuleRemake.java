package frc.robot.Swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class SwerveModuleRemake extends SwerveModule {

    private static final PIDSettings drivePIDSettings = PIDSettings.EMPTY_PID_SETTINGS;
    private static final PIDSettings turnPIDSettings = PIDSettings.EMPTY_PID_SETTINGS;

    private static final FeedForwardSettings driveFFSettings = FeedForwardSettings.EMPTY_FF_SETTINGS;
    private static final FeedForwardSettings turnFFSettings = FeedForwardSettings.EMPTY_FF_SETTINGS;

    private static final double MIN_SPEED_LIMIT = 0.1;
    private static final double DEGREES_IN_ROTATION = 360;
    private static final double SECONDS_IN_MINUTE = 60;

    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = (double) 7 / 150;
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    private static final double ENCODER_DISCONTINUITY_POINT = -1;

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;

    private final CANcoder cancoder;

    public SwerveModuleRemake(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                              boolean INVERT_DRIVE_MOTOR, boolean INVERT_TURN_MOTOR,
                              double absoluteEncoderOffset, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, INVERT_DRIVE_MOTOR, INVERT_TURN_MOTOR, absoluteEncoderOffset,
                drivePIDSettings, turnPIDSettings, driveFFSettings, turnFFSettings, MIN_SPEED_LIMIT);

        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;

        this.cancoder = cancoder;

        this.driveMotor.setInverted(INVERT_DRIVE_MOTOR);
        this.turnMotor.setInverted(INVERT_TURN_MOTOR);

        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    @Override
    protected void configureDriveController() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setEncoderConversionFactor(DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS);
        driveMotor.configureLoop(driveMotorPIDSettings, driveMotorFeedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.restoreFactoryDefaults();
        turnMotor.setPositionConversionFactor(TURN_POSITION_IN_ROTATION);
        turnMotor.setVelocityConversionFactor(TURN_VELOCITY_IN_ROTATION);
        turnMotor.configureLoop(turnMotorPIDSettings, turnMotorFeedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().
                withAbsoluteSensorDiscontinuityPoint(ENCODER_DISCONTINUITY_POINT).
                withSensorDirection(turnMotorInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(absoluteEncoderOffset / DEGREES_IN_ROTATION);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degrees));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("rotation", () -> getAbsoluteModuleAngle().getDegrees());
    }
}

