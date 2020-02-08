/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Carousel.SpinCarousel;
import frc.robot.commands.ColorSpinner.ColorSpinnerExtend;
import frc.robot.commands.ColorSpinner.ColorSpinnerPosition;
import frc.robot.commands.ColorSpinner.ColorSpinnerRetract;
import frc.robot.commands.ColorSpinner.ColorSpinnerRotation;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Intake.IntakeWithAxis;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.ShooterSparkControl;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import lib.oi.LogitechF310;
import lib.oi.Thrustmaster;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // **SUBSYSTEMS**
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Carousel carousel = new Carousel();
  private final Turret turret = new Turret();
  private final ColorSpinner colorSpinner = new ColorSpinner();

  // **JOYSTICKS**
  LogitechF310 controller = new LogitechF310(Constants.ControllerConstants.USB_CONTROLLER);
  Thrustmaster leftStick = new Thrustmaster(Constants.ControllerConstants.USB_LEFT_STICK);
  Thrustmaster rightStick = new Thrustmaster(Constants.ControllerConstants.USB_RIGHT_STICK);

  // **COMMANDS**
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // DRIVE
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);

  // INTAKE
  private final SetIntakeSpeed setIntakeSpeed = new SetIntakeSpeed(intake, Constants.IntakeConstants.SPEED);
  private final IntakeWithAxis intakeWithAxis = new IntakeWithAxis(intake, controller);

  // COLOR SPINNER
  private final ColorSpinnerRotation colorSpinnerRotation = new ColorSpinnerRotation(colorSpinner);
  private final ColorSpinnerPosition colorSpinnerPosition = new ColorSpinnerPosition(colorSpinner);
  private final ColorSpinnerExtend colorSpinnerExtend = new ColorSpinnerExtend(colorSpinner);
  private final ColorSpinnerRetract colorSpinnerRetract = new ColorSpinnerRetract(colorSpinner);

  // SHOOTER
  private final ShooterSparkControl shooterSparkControl = new ShooterSparkControl(shooter, 4000);

  // CAROUSEL
  private final SpinCarousel spinCarousel = new SpinCarousel(carousel);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive.setDefaultCommand(driveWithJoysticks);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // COLOR SPINNER
    // leftStick.left.whenPressed(colorSpinnerRotation);
    // leftStick.right.whenPressed(colorSpinnerPosition);
    // rightStick.right.whileHeld(colorSpinnerExtend);
    // rightStick.left.whileHeld(colorSpinnerRetract);

    // SHOOTER
    leftStick.middle.whileHeld(shooterSparkControl);

    // INTAKE 
    controller.x.whileHeld(setIntakeSpeed);
    controller.lb.whileHeld(intakeWithAxis);
  }

  /*
   * private Command getAutoFromSwitches() {
   * 
   * }
   */

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
