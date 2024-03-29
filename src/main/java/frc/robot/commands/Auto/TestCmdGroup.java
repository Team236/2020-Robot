/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TestCmdGroup extends SequentialCommandGroup {
  /**
   * Creates a new TestCmdGroup.
   */
  public TestCmdGroup(Drive drive) {
    // Add your commands in the super() call, e.g.
    // super(new SparkMotnControl(drive, 12, 1), new FollowProfile(RobotContainer.tes, false));
  }
}
