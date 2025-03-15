// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Quaternion;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private final PhotonCamera front_camera;
    private final PhotonCamera rear_camera;
    private final PhotonPoseEstimator front_photonEstimator;
    private final PhotonPoseEstimator rear_photonEstimator;
    private AprilTagFieldLayout kTagLayout;
    private final Field2d field = new Field2d();
    private Pose2d lastPose = new Pose2d();
    private PhotonPipelineResult lastResult = null;
    private boolean updateDashboard = true;


    public VisionSubsystem() {
        assert (instance == null);
        instance = this;
 
        front_camera = new PhotonCamera("");
        rear_camera = new PhotonCamera("");

        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        front_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(new Translation3d(Inches.of(0.25), Inches.of(13), Inches.of(32.87)), new Rotation3d(new Quaternion(0.349066 , 1, 0, 0))));
        front_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rear_photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        new Transform3d(new Translation3d(Inches.of(0.25), Inches.of(13), Inches.of(32.87)), new Rotation3d(new Quaternion(0.349066 , 1, 0, 0))));
        rear_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putData("Vision/Field", field);
    }

    public static VisionSubsystem getInstance() {
        return instance;
    }

    public boolean updatePoseEstimate(PoseEstimator<SwerveModulePosition[]> poseEstimator) {
        List<PhotonPipelineResult> front_results = front_camera.getAllUnreadResults();
        List<PhotonPipelineResult> rear_results = rear_camera.getAllUnreadResults();

        boolean newResult = !front_results.isEmpty();
        boolean updated = false;

        // TODO Consider updating standard deviations

        for (PhotonPipelineResult result : front_results) {
            lastResult = result;

            Optional<EstimatedRobotPose> optRobotPose = front_photonEstimator.update(result);
            if (!optRobotPose.isPresent()) {
                continue;
            }

            EstimatedRobotPose robotPose = optRobotPose.get();
            lastPose = robotPose.estimatedPose.toPose2d();
            field.setRobotPose(lastPose);

          
            if (poseEstimator != null) {
                poseEstimator.addVisionMeasurement(lastPose, robotPose.timestampSeconds);
                updated = true;
            }
        }

        for (PhotonPipelineResult result : rear_results) {
            lastResult = result;

            Optional<EstimatedRobotPose> optRobotPose = rear_photonEstimator.update(result);
            if (!optRobotPose.isPresent()) {
                continue;
            }

            EstimatedRobotPose robotPose = optRobotPose.get();
            lastPose = robotPose.estimatedPose.toPose2d();
            field.setRobotPose(lastPose);

          
            if (poseEstimator != null) {
                poseEstimator.addVisionMeasurement(lastPose, robotPose.timestampSeconds);
                updated = true;
            }
        }

        if (updateDashboard) {
            SmartDashboard.putBoolean("Vision/New result", newResult);
        }

        return updated;
    }

    @Override
    public void periodic() {
        boolean haveTarget = lastResult != null && lastResult.hasTargets();

        // TODO Belongs in periodic() or elsewhere?
        if (lastResult != null) {
        
        }
       

        
    }
}