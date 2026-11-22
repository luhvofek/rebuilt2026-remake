package frc.robot.Swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SwerveModuleHolder {

    private static final String NAMESPACE_NAME = "module holder";

    private static final RootNamespace namespace = new RootNamespace(NAMESPACE_NAME);

    private static final CANBus canivore = new CANBus("canivore");

    private static final String FRONT_LEFT_NAMESPACE_NAME = "front left";
    private static final String FRONT_RIGHT_NAMESPACE_NAME = "front right";
    private static final String BACK_LEFT_NAMESPACE_NAME = "back left";
    private static final String BACK_RIGHT_NAMESPACE_NAME = "back right";

    private static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    private static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    private static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    private static final boolean BACK_RIGHT_DRIVE_INVERTED = true;

    private static final boolean FRONT_LEFT_CANCODER_INVERTED = false;
    private static final boolean FRONT_RIGHT_CANCODER_INVERTED = false;
    private static final boolean BACK_LEFT_CANCODER_INVERTED = false;
    private static final boolean BACK_RIGHT_CANCODER_INVERTED = false;

    private static final double FRONT_LEFT_OFFSET = 61.640625;
    private static final double FRONT_RIGHT_OFFSET = 207.50976562;
    private static final double BACK_LEFT_OFFSET = 162.10546875;
    private static final double BACK_RIGHT_OFFSET = 168.04101563;

    private static SwerveModuleRemake frontLeft;
    private static SwerveModuleRemake frontRight;
    private static SwerveModuleRemake backLeft;
    private static SwerveModuleRemake backRight;

    public static SwerveModuleRemake getFrontLeft() {
        if (frontLeft == null) {
            frontLeft = new SwerveModuleRemake(FRONT_LEFT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_FRONT_LEFT_DRIVE_TALON_FX_ID, canivore),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_FRONT_LEFT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), FRONT_LEFT_DRIVE_INVERTED, FRONT_LEFT_CANCODER_INVERTED
                    , FRONT_LEFT_OFFSET, new CANcoder(RobotMap.CAN.SWERVE_FRONT_LEFT_ABSOLUTE_ENCODER_ID, canivore));
        }
        return frontLeft;
    }

    public static SwerveModuleRemake getFrontRight() {
        if (frontRight == null) {
            frontRight = new SwerveModuleRemake(FRONT_RIGHT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_FRONT_RIGHT_DRIVE_TALON_FX_ID, canivore),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_FRONT_RIGHT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), FRONT_RIGHT_DRIVE_INVERTED,
                    FRONT_RIGHT_CANCODER_INVERTED, FRONT_RIGHT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_FRONT_RIGHT_ABSOLUTE_ENCODER_ID, canivore));
        }
        return frontRight;
    }

    public static SwerveModuleRemake getBackLeft() {
        if (backLeft == null) {
            backLeft = new SwerveModuleRemake(BACK_LEFT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_BACK_LEFT_DRIVE_TALON_FX_ID, canivore),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_BACK_LEFT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), BACK_LEFT_DRIVE_INVERTED,
                    BACK_LEFT_CANCODER_INVERTED, BACK_LEFT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_BACK_LEFT_ABSOLUTE_ENCODER_ID, canivore));
        }
        return backLeft;
    }

    public static SwerveModuleRemake getBackRight() {
        if (backRight == null) {
            backRight = new SwerveModuleRemake(BACK_RIGHT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_BACK_RIGHT_DRIVE_TALON_FX_ID, canivore),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_BACK_RIGHT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), BACK_RIGHT_DRIVE_INVERTED,
                    BACK_RIGHT_CANCODER_INVERTED, BACK_RIGHT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_BACK_RIGHT_ABSOLUTE_ENCODER_ID, canivore));
        }
        return backRight;
    }

    public void updateNamespace(){
        namespace.update();
    }
}
