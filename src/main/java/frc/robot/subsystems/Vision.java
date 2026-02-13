// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.Camera;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;
import org.photonvision.proto.Photon.ProtobufMultiTargetPNPResult;

public class Vision extends SubsystemBase {
/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */

    private final NetworkTableInstance networkTable = NetworkTableInstance
    .getDefault()
    .getTable("SwerveSubsystem")
    .getInstance();
  private final StructPublisher<Transform3d> publisher = networkTable
    .getStructTopic("AprilTagPose3D", Transform3d.struct)
    .publish();
  private final StructPublisher<Pose3d> publisher2 = networkTable
    .getStructTopic("GoalPose", Pose3d.struct)
    .publish();
  private final StructEntry<Pose2d> pubPose = networkTable
    .getStructTopic("SwervePose", Pose2d.struct)
    .getEntry(new Pose2d());

  public Vision(){
    boolean hasTargets;
    //pubPose.set(pubPose.get());
  }

  /**
   * April Tag Field Layout of the year.
   */

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
    AprilTagFields.k2025ReefscapeWelded
  );
  public VisionSystemSim visionSim;
  private Supplier<Pose2d> currentPose;
  private Field2d field2d;
  private SwerveDrive swerve;


  PhotonCamera waffleCam = new PhotonCamera("waffleCam");
  Transform3d camToRobot = new Transform3d(0,0,0, new Rotation3d(0, 0, 0));
  PhotonPoseEstimator estimator = new PhotonPoseEstimator(fieldLayout,camToRobot);
  SwerveDrive drive;

    public List <PhotonPipelineResult> results(){
      var results = waffleCam.getAllUnreadResults();
      return results;
    

    }
    
    public void setSwerveDrive(SwerveDrive drive){
      this.drive = drive;
    }

    public void addVisionMeasurement(){
      Optional<EstimatedRobotPose> estPose = this.getEstimatedPose();
      if (estPose.isPresent()){
        EstimatedRobotPose pose = estPose.get();
        System.out.println("pose " + pose.estimatedPose);
        System.out.println("time " + pose.timestampSeconds);
        pubPose.set(pose.estimatedPose.toPose2d());
    //     private final StructPublisher<Pose2d> pubPose = networkTable
    // .getStructTopic("fuck you", Pose2d.struct)
    // .publish();
        drive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        drive.addVisionMeasurement(null, 0, null);
      }
    }
    public  Optional<EstimatedRobotPose> getEstimatedPose() {
      return estimator.estimateAverageBestTargetsPose(waffleCam.getLatestResult());

    }
    @Override
    public void periodic() {
      this.addVisionMeasurement();
      System.out.println("Doing some stuff");
      var result = waffleCam.getLatestResult();
      boolean hasTargets = result.hasTargets();
      System.out.println("IsHasTarget: " + hasTargets);
      if(hasTargets){
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();

      System.out.println("Yaw: " + yaw);
      System.out.println("Pitch: " + pitch);
      System.out.println("Area: " + area);
      System.out.println("TargetID: " + targetID);
      System.out.println("PoseAmbiguity" + poseAmbiguity);
      } else {
        System.out.println("There is no Target");
      }
    }
  }
