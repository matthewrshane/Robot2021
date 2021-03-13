// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Drives the robot towards the closest powercell for use in the Galactic Search challenge. */
public class DriveToPowercellCommand extends CommandBase {
  // Subsystems

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final VisionSubsystem m_visionSubsystem;

  /** Initializes this command. */
  public DriveToPowercellCommand(
      DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run when the command is executed. */
  @Override
  public void execute() {
    // Set the neutral mode of the drive motors to break, for more responsiveness.
    m_drivetrainSubsystem.setNeutralMode(NeutralMode.Brake);

    // Check if there is a target detected.
    if (!m_visionSubsystem.hasTargets()) return;

    // Locate the nearest powercell.
    double rotationToTarget = m_visionSubsystem.getRotationToTarget();

    // Check if we're facing the nearest powercell.
    if (Math.abs(rotationToTarget) >= VisionConstants.kRotationTolerance) {
      // Turn towards the nearest powercell.
      m_drivetrainSubsystem.arcadeDrive(
          0, m_visionSubsystem.getNonlinearRotationalSpeed(rotationToTarget));
      return;
    }

    // Get the distance to the powercell.
    double distanceToTarget = m_visionSubsystem.getDistanceToTarget();

    // Check if the robot is close to the powercell.
    if (distanceToTarget >= VisionConstants.kDistancePowercellMinimum) {
      // Drive forward towards the nearest powercell.
      m_drivetrainSubsystem.arcadeDrive(
          m_visionSubsystem.getNonlinearSpeed(distanceToTarget, Units.feetToMeters(5)), 0);
      return;
    }
  }
}
