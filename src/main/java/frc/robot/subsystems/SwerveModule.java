package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.helper.IDashboardProvider;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements IDashboardProvider{
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;
    private final String name;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {
        this.registerDashboard();
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        this.driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        this.driveMotor.setInverted(driveMotorReversed);
        this.turningMotor.setInverted(turningMotorReversed);

        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.turningMotor.setIdleMode(IdleMode.kBrake);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turningEncoder = new CANCoder(absoluteEncoderId);
        this.turningEncoder.configFactoryDefault();

        this.driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        this.driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        this.turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        this.turningEncoder.configSensorDirection(false);
        this.turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        this.turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        this.turningPidController.enableContinuousInput(-180,180);

        this.name = name;
    }

    public double getTurningPosition() {
        return this.turningEncoder.getAbsolutePosition() - this.absoluteEncoderOffset;
    }

    public SwerveModulePosition getSwerverPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromDegrees(this.getTurningPosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(this.getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        SwerveModuleState states = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(states.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(Math.IEEEremainder(this.getState().angle.getDegrees(), 360), states.angle.getDegrees()));
        SmartDashboard.putNumber("Degrees" + this.name, Math.IEEEremainder(this.getState().angle.getDegrees(), 360));
        SmartDashboard.putNumber("Target Degrees" + this.name, states.angle.getDegrees());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.name + "TurnVelocity", this.turningEncoder.getVelocity());
        SmartDashboard.putNumber(this.name + "TurnPostion", this.getTurningPosition());
    }
}
