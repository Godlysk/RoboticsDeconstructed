/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DrivePIDCommand;
import frc.robot.commands.MoveByAngleCommand;
import frc.robot.commands.MoveByDistanceCommand;
import frc.robot.commands.SolenoidForwardCommand;
import frc.robot.commands.SolenoidReverseCommand;
import frc.robot.commands.SpeedBoostCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final SolenoidSubsystem solenoidSubsystem = new SolenoidSubsystem();

  // Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final DrivePIDCommand drivePIDCommand = new DrivePIDCommand(driveSubsystem);
  private final SolenoidForwardCommand solenoidForwardCommand = new SolenoidForwardCommand(solenoidSubsystem);

  // IO Devices
  public static Joystick joy1 = new Joystick(1);
  public static Encoder enc_L = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  public static Encoder enc_R = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    solenoidSubsystem.setDefaultCommand(solenoidForwardCommand);
    driveSubsystem.setDefaultCommand(drivePIDCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton commandBrakeButton = new JoystickButton(joy1, Constants.brakeButtonNumber);
    commandBrakeButton.whenPressed(new BrakeCommand(driveSubsystem));

    JoystickButton commandBoostButton = new JoystickButton(joy1, Constants.boostButtonNumber);
    commandBoostButton.whenPressed(new SpeedBoostCommand());

    JoystickButton commandSolenoidButton = new JoystickButton(joy1, Constants.solenoidButtonNumber);
    commandSolenoidButton.toggleWhenPressed(new SolenoidReverseCommand(solenoidSubsystem));

  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new MoveByDistanceCommand(driveSubsystem, 5);
    // return new MoveByAngleCommand(driveSubsystem, 30);
    return null;
  }


  public static double getY(Joystick joy, double deadband) {
    
    double value = -1 * joy.getY();
    if (Math.abs(value) < deadband) return 0;
    return value;

  }

  public static double getZ(Joystick joy, double deadband) {
    
    double value = joy.getZ();
    if (Math.abs(value) < deadband) return 0;
    return value;

  }
  
}
