/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.ClimberConstants.*;

public class ClimbPositionControl extends CommandBase {
  private Climber climber;
  private double targetPosition, margin;
  /**
   * Creates a new ClimbPositionControl.
   */
  public ClimbPositionControl(Climber climber, double position, double margin) {
    this.climber = climber;
    addRequirements(this.climber);
    this.targetPosition = position;
    this.margin = margin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setP(kP);    
    climber.setI(kI);
    climber.setD(kD);
    climber.setFF(kFF);
    climber.setOutputRange(min, max);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSetPoint(targetPosition);

    SmartDashboard.putNumber("climb setpoint", targetPosition);
    SmartDashboard.putNumber("actual position", climber.getEncoderPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.getEncoderPosition() - targetPosition) < margin;
  }
}
