package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final GamepadJoystick joystick = new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);
    private final SwerveJoystickCmd swerveJoystickCmd = new SwerveJoystickCmd(swerveSubsystem, joystick);

    public RobotContainer() {
        this.swerveSubsystem.setDefaultCommand(this.swerveJoystickCmd);
    }

    public Command getAutonomousCommand() {
        return null;
    }
        
}