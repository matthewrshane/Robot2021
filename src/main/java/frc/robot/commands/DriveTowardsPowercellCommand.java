// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.GalacticSearchConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveTowardsPowercellCommand extends CommandBase {

  // Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final VisionSubsystem m_visionSubsystem;

  // Is the command finished.
  private boolean m_finished;

  // Is the command functioning properly.
  private boolean m_functioning;

  /** Initializes this command. */
  public DriveTowardsPowercellCommand(
      DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;

    addRequirements(m_drivetrainSubsystem);
    addRequirements(m_visionSubsystem);
  }

  /** This method is run once when the command is first executed. */
  @Override
  public void initialize() {
    m_finished = false;
    m_functioning = true;
  }

  /** This method is run continously while the command is executed. */
  @Override
  public void execute() {
    if (!m_visionSubsystem.hasTargets()) {
      m_functioning = false;
      return;
    }

    // Locate the nearest powercell and its distance.
    double targetDistance = m_visionSubsystem.getDistanceToTarget();
    System.out.println(targetDistance);

    // Check if the robot is close to the powercell.
    if (targetDistance >= GalacticSearchConstants.kDrivePowercellDistanceApproach) {
      // Drive forward towards the nearest powercell.
      m_drivetrainSubsystem.arcadeDrive(
          m_visionSubsystem.getNonlinearSpeed(targetDistance, Units.feetToMeters(5), 5.0), 0);
      return;
    }
    m_finished = true;
  }

  /** This method is run to check whether the command is finished. */
  @Override
  public boolean isFinished() {
    return m_finished;
  }

  /**
   * This method is run to check whether the command is functioning properly. If not, the previous
   * command must be reverted to.
   */
  public boolean isFunctional() {
    return m_functioning;
  }
}
