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
    private final double absoluteEncoderOffsetRad;
    private final String name;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {
        this.registerDashboard();
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
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
        resetEncoders();
    }

    public double getDrivePosition() {
        return this.driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return this.turningEncoder.getAbsolutePosition();
    }

    public double getTurningAngle() {
        return this.getTurningPosition();
    }

    public SwerveModulePosition getSwerverPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            new Rotation2d(this.getTurningPosition())
        );
    }

    public double getDriveVelocity() {
        return this.driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return this.turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderangle() {
        double angle = this.turningEncoder.getBusVoltage();
        return angle * (this.absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        this.driveEncoder.setPosition(0);
        this.turningEncoder.setPosition(getAbsoluteEncoderangle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.getDriveVelocity(),
            new Rotation2d(this.getTurningPosition())
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
        SmartDashboard.putNumber(this.name + "TurnSpeed", this.turningEncoder.getVelocity());
        SmartDashboard.putNumber(this.name + "TurnAngle", this.getTurningAngle());
    }
}
