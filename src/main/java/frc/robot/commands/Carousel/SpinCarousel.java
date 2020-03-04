/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Carousel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;

public class SpinCarousel extends CommandBase {

  private Carousel carousel;
  private boolean isRev;

  /**
   * Spins carousel at speed specified in Constants
   */
  public SpinCarousel(Carousel carousel, boolean isRev) {
    this.isRev = isRev;
    this.carousel = carousel;

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(this.carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isRev) {
      carousel.setSpeed(-Constants.CarouselConstants.INTAKE_SPEED);
    } else {
      carousel.setSpeed(Constants.CarouselConstants.INTAKE_SPEED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    carousel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
