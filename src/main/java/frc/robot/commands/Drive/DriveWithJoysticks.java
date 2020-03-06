/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DriveWithJoysticks extends CommandBase {

  private Drive drive;
  private Joystick leftStick, rightStick;
  private boolean isCube;
  private int pow = 1;
  private boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;

  /**
   * Creates a new DriveWithJoysticks.
   */
  public DriveWithJoysticks(Drive drive, Joystick leftStick, Joystick rightStick, boolean isCube) {

    this.drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;

    this.isCube = isCube;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isCube) {
      this.pow = 3;
      this.isDeadzone = false;
    } else {
      this.pow = 1;
      this.isDeadzone = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // IS_DEADZONE determines whether joystick deadzone is considered
    if (this.isDeadzone) {
      drive.setLeftSpeedWithDeadzone(Math.pow(-leftStick.getY(), pow));
      drive.setRightSpeedWithDeadzone(Math.pow(-rightStick.getY(), pow));
    } else {
      drive.setLeftSpeed(Math.pow(-leftStick.getY(), pow));
      drive.setRightSpeed(Math.pow(-rightStick.getY(), pow));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
