/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class SparkMotnControl extends CommandBase {

  private Drive drive;

  private double dist;
  private double margin;
  private double error;

  private double kP, kI, kD, kFF;

  /**
   * Creates a new SparkMotnControl.
   */
  public SparkMotnControl(Drive drive, double dist, double margin) {
    this.drive = drive;
    addRequirements(this.drive);

    this.dist = dist;
    this.margin = margin;

    this.kP = Constants.DriveConstants.kP;
    this.kI = Constants.DriveConstants.kI;
    this.kD = Constants.DriveConstants.kD;
    this.kFF = Constants.DriveConstants.kF;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();

    drive.setkP(this.kP);
    drive.setkI(this.kI);
    drive.setkD(this.kD);
    drive.setkF(this.kFF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setOutputRange(Constants.DriveConstants.MIN_OUTPUT, Constants.DriveConstants.MAX_OUTPUT);
    drive.setSetPoint(dist);
    
    error = Math.abs(dist - drive.getLeftEncoder());
    // error = dist - drive.getLeftEncoder();

    SmartDashboard.putNumber("Spark setPoint", dist);
    SmartDashboard.putNumber("spark error", error);
    SmartDashboard.putNumber("Spark L encoder", drive.getLeftEncoder());
    SmartDashboard.putNumber("Spark R encoder", drive.getRightEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDistMargin = error < margin;
    boolean isSpeedMargin = drive.getLeftSpeed() < 1.0;

    // Returns true when speed and distance are within acceptable margins
    // return isDistMargin && isSpeedMargin;
    return false;
  }
}
