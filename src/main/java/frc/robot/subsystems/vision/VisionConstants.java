// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {

  public static boolean loadHomeField = true;
  public static String homeFieldFileName = "whshallway.json";
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "arducam_camera_0";
  public static String camera1Name = "arducam_camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(-7),
          Units.inchesToMeters(-1.5),
          Units.inchesToMeters(20),
          new Rotation3d(0.0, 0.0, Math.PI));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-2),
          Units.inchesToMeters(-11.0),
          Units.inchesToMeters(17),
          new Rotation3d(0.0, 0.0, 3 * Math.PI / 2));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Initialization reliability thresholds: when using the very first vision
  // measurement to initialize odometry, require the reported measurement
  // uncertainty to be reasonably small so we don't initialize to a bad pose.
  // Maximum allowed linear std dev (meters) for initialization
  public static double visionInitMaxLinearStdDevMeters = 0.5;
  // Maximum allowed angular std dev (radians) for initialization
  public static double visionInitMaxAngularStdDevRadians = Math.toRadians(45.0);

  // Proximity thresholds: only accept vision updates when the reported vision pose
  // is reasonably close to the current odometry estimate. These prevent large
  // jumps when vision or odometry briefly disagree. Used by manual apply.
  // Maximum allowed translation difference (meters)
  public static double maxVisionPoseDistanceMeters = 1.0;
  // Maximum allowed rotation difference (radians)
  public static double maxVisionPoseAngleRadians = Math.toRadians(30.0);

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.1; // Meters
  public static double angularStdDevBaseline = 0.1; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  static {
    if (loadHomeField) {
      try {
        // Full path to the JSON file
        String jsonFilePath = Filesystem.getDeployDirectory() + "/" + homeFieldFileName;

        System.out.println("Loading AprilTag layout from path: " + jsonFilePath);

        aprilTagLayout = new AprilTagFieldLayout(jsonFilePath);

        System.out.println(homeFieldFileName + " AprilTag layout loaded successfully.");
      } catch (Exception e) {
        System.err.println("Error loading AprilTag layout: " + e.getMessage());
        e.printStackTrace();
        aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        System.out.println("Reefscape AndyMark AprilTag layout loaded successfully.");
      }
    } else {
      aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
      System.out.println("Reefscape AndyMark AprilTag layout loaded successfully.");
    }
  }
}
