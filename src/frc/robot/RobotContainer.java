/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SteeringTuning;
import frc.robot.commands.SteeringToAngle;
import frc.robot.commands.CalibrateDist;
import frc.robot.commands.DriveTuning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @SuppressWarnings("unused")
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final CommandJoystick m_joystick = new CommandJoystick(0);

  private final CommandXboxController m_xbox = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> m_joystick.getX(),
    () -> -m_joystick.getY(), () -> m_joystick.getZ(), true));

    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> m_xbox.getLeftX(),
        () -> -m_xbox.getLeftY(), () -> m_xbox.getRightX(), true));
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_joystick.button(11).onTrue(new CalibrateDist(m_driveSubsystem));
    m_joystick.button(7).onTrue(new SteeringToAngle(m_driveSubsystem, 0));
    m_joystick.button(8).onTrue(new SteeringToAngle(m_driveSubsystem, 90));
    m_joystick.button(9).onTrue(new SteeringToAngle(m_driveSubsystem, 180));
    m_joystick.button(10).onTrue(new SteeringToAngle(m_driveSubsystem, 270));
    m_joystick.button(11).onTrue(new CalibrateDist(m_driveSubsystem));
    //m_joystick.button(11).onTrue(new DriveTuning(m_driveSubsystem));
    m_joystick.button(12).onTrue(new SteeringTuning(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
