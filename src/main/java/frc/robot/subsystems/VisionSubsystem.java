// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Represents the vision subsystem.
 *
 * <p>The vision subsystem encapsulates managing cameras attached to the roboRIO, and interfacing
 * with coprocessors to provide object tracking data from the other camera.
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  @Log.CameraStream(
      name = "Camera",
      width = 7,
      height = 6,
      rowIndex = 0,
      columnIndex = 0,
      tabName = "Driver View")
  // Photonvision Camera
  private PhotonCamera m_camera = new PhotonCamera("longwood564");

  /** Initializes the vision subsystem. */
  public VisionSubsystem() {
    // If we're runnning on the real robot, enable the camera.
    if (RobotBase.isReal()) {
      // TODO: Not necessary?
    }
  }

  private PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  public boolean hasTargets() {
    return getLatestResult().hasTargets();
  }

  /**
   * Gets the 
   * @return
   */
  public PhotonTrackedTarget getTarget() {
    return hasTargets() ? getLatestResult().getBestTarget() : null;
  }

  /**
   * Gets the rotation from the front-center of the robot, where the camera is, to the target. Positive values indicate a counter-clockwise rotation, to the left.
   * @return the rotation, or null if there is no tracked target.
   */
  public double getRotationToTarget() {
    return hasTargets() ? getTarget().getYaw() : null;
  }

  public double getPitchToTarget() {
    return hasTargets() ? getTarget().getPitch() : null;
  }

  public double getDistanceToTarget() {
    return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kHeightCameraLow, VisionConstants.kHeightTarget, Units.degreesToRadians(VisionConstants.kPitchCameraLow), Units.degreesToRadians(getPitchToTarget()));
  }

}
