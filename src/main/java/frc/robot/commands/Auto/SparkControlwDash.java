/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class SparkControlwDash extends CommandBase {

  private Drive driveSub;
  private double dist;
  private double margin;
  private double error;
  private double ff, pp, ii, dd;
  private double kF, kP, kI, kD;
  private double lastf, lastp, lasti, lastd;
  /**
   * Creates a new SparkMotnControl.
   */
  public SparkControlwDash(Drive driveSub, double dist, double margin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    addRequirements(this.driveSub);
    kF = 0; kP = 0; kI=0; kD=0;

    SmartDashboard.putNumber("Feed Fwd", lastf);
    SmartDashboard.putNumber("k_P", lastp);
    SmartDashboard.putNumber("k_I", lasti);
    SmartDashboard.putNumber("k_D", lastd);
    this.dist = dist;
    this.margin = margin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.resetEncoders();

    driveSub.setkP(kP);
    driveSub.setkI(kI);
    driveSub.setkD(kD);
    driveSub.setkF(kF);
 //shooterSub.setP(Constants.AutoConstants.kP);
  // shooterSub.setI(Constants.ShooterConstants.kI);
  // shooterSub.setD(Constants.ShooterConstants.kD);
  //shooterSub.setFF(Constants.ShooterConstants.kFF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ff = SmartDashboard.getNumber("Feed Fwd", 0);
    pp = SmartDashboard.getNumber("k_P", 0);
    ii = SmartDashboard.getNumber("k_I", 0);
    dd = SmartDashboard.getNumber("k_D", 0);
  
     if ((ff != kF)) {driveSub.setkF(ff); kF = ff;}
     SmartDashboard.putNumber("set ff", ff);
     if ((pp != kP)) {driveSub.setkP(pp); kP = pp;}
     SmartDashboard.putNumber("set pp", pp);
     if ((ii != kI)) {driveSub.setkI(ii); kI = ii;}
     SmartDashboard.putNumber("set ii", ii);
     if ((dd != kD)) {driveSub.setkD(dd); kD = dd;}
     SmartDashboard.putNumber("set dd", dd);

    driveSub.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

    driveSub.setSetPoint(dist);

    // Prints useful info to dashboard
    SmartDashboard.putNumber("Spark setPoint", (dist*Constants.DriveConstants.REV_TO_IN_K));
    SmartDashboard.putNumber("Spark L encoder", driveSub.getLeftEncoder());

    SmartDashboard.putNumber("Spark R encoder", driveSub.getRightEncoder());

    error = Math.abs(dist*Constants.DriveConstants.REV_TO_IN_K - driveSub.getLeftEncoder());

    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("margin", margin);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
    lastf = SmartDashboard.getNumber("Feed Fwd", 0);
    lastp = SmartDashboard.getNumber("k_P", 0);
    lasti = SmartDashboard.getNumber("k_I", 0);
    lastd = SmartDashboard.getNumber("k_D", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDistMargin = error < margin;
   // boolean isSpeedMargin = driveSub.getLeftSpeed() < 1.0;

    // Returns true when speed and distance are within acceptable margins
    return isDistMargin; // && isSpeedMargin;
  }
}
