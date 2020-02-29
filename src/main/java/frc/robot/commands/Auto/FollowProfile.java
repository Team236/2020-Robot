/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import lib.motionProfile.DriveParameters;
import lib.motionProfile.Element;
import lib.motionProfile.TrapProfile;

public class FollowProfile extends CommandBase {

  private Drive drive;

  TrapProfile profile;
  DriveParameters params;
  boolean reverse;
  int reverseMultiplier = 1;
  double endPos, margin, leftDistToEnd, rightDistToEnd;
  public static boolean isDebug = true;
  int i = 0;

  /**
   * Creates a new FollowProfile.
   */
  public FollowProfile(Drive drive, TrapProfile _p, DriveParameters _params, boolean _rev) {
    this.drive = drive;
    addRequirements(this.drive);

    this.profile = _p;
    this.params = _params;
    this.reverse = _rev;
    this.margin = profile.margin;
    this.endPos = profile.getLast().position;

    if (reverse) {
      this.reverseMultiplier = -1;
    }

  }

  /*
   * public FollowProfile(TrapProfile _p) { this(_p,
   * Constants.DriveConstants.DRIVE_PARAMS, false); }
   * 
   * public FollowProfile(TrapProfile _p, boolean _rev) { this(_p,
   * RobotMap.DriveMap.DRIVE_PARAMS, _rev); }
   * 
   * public FollowProfile(ProfileParameters _params, DriveParameters _driveParams,
   * boolean _rev) { this(new TrapProfile(_params), _driveParams, _rev); }
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    drive.resetEncoders();
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Constrain i within bounds of profile
    if (i < 0) {
      i = 0;
    } else if (i >= profile.length()) {
      i = profile.length() - 1;
    }

    // Get element from profile
    Element e = profile.get(i);

    // Calculate feedforwards
    double l_v = params.kV_l * e.speed;
    double l_a = params.kA_l * e.acceleration;
    // Calculate corrections
    double l_error = (reverseMultiplier * e.position - drive.getLeftDistance());
    double l_p = params.kP * l_error;
    leftDistToEnd = reverseMultiplier * drive.getLeftDistance() - endPos;

    // Repeat for right side
    // Calculate feedforwards
    double r_v = params.kV_r * e.speed;
    double r_a = params.kA_r * e.acceleration;
    // Calculate corrections
    double r_error = (reverseMultiplier * e.position - drive.getRightDistance());
    double r_p = params.kP * r_error;
    rightDistToEnd = reverseMultiplier * drive.getRightDistance() - endPos;

    drive.setLeftSpeed(reverseMultiplier * (l_v + l_a + l_p));
    drive.setRightSpeed(reverseMultiplier * (r_v + r_a + r_p));

    i++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isLeftFinished = Math.abs(leftDistToEnd) < margin;
    boolean isRightFinished = Math.abs(rightDistToEnd) < margin;

    return isLeftFinished && isRightFinished && (i == profile.length());
  }
}
