package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Timer;
import java.util.TimerTask;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    TimerTask task;

    public PhotonCameraWrapper() {
        // Set up a test arena of two apriltags at the center of each driver station set
        final AprilTag tag18 =
                new AprilTag(
                        18,
                        new Pose3d(
                                new Pose2d(
                                        FieldConstants.length,
                                        FieldConstants.width / 2.0,
                                        Rotation2d.fromDegrees(180))));
        final AprilTag tag04 =
                new AprilTag(
                        04,
                        new Pose3d(new Pose2d(0.0, 0, new Rotation2d(0, 0))));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        // atList.add(tag18);
        atList.add(tag04);

        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout atfl =
                new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

        // Forward Camera
        Timer timer = new Timer();
        
        task = new TimerTask() {
            public void run() {
                System.out.println("Connecting to photon camera '" + VisionConstants.cameraName + "'");
                photonCamera =
                        new PhotonCamera(
                                VisionConstants
                                        .cameraName); // Change the name of your camera here to whatever it is in the
                System.out.println("Connected camera, " + photonCamera.getName() + ": " + photonCamera.isConnected());
            }
        };
        
        task.run();

        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        atfl, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, VisionConstants.robotToCam);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public List<PhotonTrackedTarget> getTargets() {
        var result = photonCamera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets;
    }
}