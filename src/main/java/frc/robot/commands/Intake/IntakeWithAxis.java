/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import lib.oi.LogitechF310;

public class IntakeWithAxis extends CommandBase {

  private Intake intake;
  private LogitechF310 controller;

  /**
   * Sets intake speed using right controller x-axis
   */
  public IntakeWithAxis(Intake intake, LogitechF310 controller) {
    this.intake = intake;
    addRequirements(this.intake);

    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSpeed(controller.getRightX());
    SmartDashboard.putNumber("intake speed", controller.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
