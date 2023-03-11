package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;
import java.util.Stack;
import java.util.Timer;
import java.util.TimerTask;
import java.util.regex.MatchResult;
import java.util.regex.Pattern;
import java.util.stream.Collector;
import java.util.stream.Stream;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;



public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    TimerTask task;

    public static final AprilTag tag04 =
            new AprilTag(
                    04,
                    new Pose3d(new Pose2d(0, 0, new Rotation2d(0))));

    ArrayList<AprilTag> tagList = new ArrayList<AprilTag>();
    public PhotonCameraWrapper() {
        /*
         * All Apriltag data is to center of tags
         * Coodinate system considers (0, 0, 0)
         *  as center of the field on ground
         */
        /*
        double size = 152.4; //in mm
        Stream<Object> ids = reader.findAll(Pattern.compile("(?:\"id\"\\s*:\\s*)\\w+")).map(x -> x.toString());
        int[] idStrings = ids.mapToInt(x -> Integer.parseInt( (x.toString() ) )).toArray();
        Stream<Object> transforms = reader.findAll(Pattern.compile("(?:\"transform\"\\s*:\\s*\\[)((?:\\s*).+(?:\\s*))+?\\]")).map(x -> x.toString());
        double[][] matrix =
                transforms.map((x) -> ((String)x).trim().split(","))
                .map(x -> Arrays.stream(x).mapToDouble(Double::parseDouble).toArray())
                .toArray(double[][]::new);
        for (int i = 0; i < matrix.length; i++) {
                double[] m = matrix[i];
                Translation3d trans = new Translation3d( m[3],  m[7], m[11] );

                //then we have to convert the affine matrix to a quaternion

                double w = Math.sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
	        double w4 = (4.0 * w);
	        double x = (m[6] - m[9]) / w4 ;
	        double y = (m[8] - m[2]) / w4 ;
	        double z = (m[1] - m[4]) / w4 ;
                Rotation3d rot = new Rotation3d(new Quaternion(w, x, y, z));
                tagList.add(new AprilTag(idStrings[i], new Pose3d(trans, rot)));

        }
        */
        /*
               // Set up a test arena of two apriltags at the center of each driver station set
        final AprilTag tag18 =
                new AprilTag(
                        18,
                        new Pose3d(
                                new Pose2d(
                                        FieldConstants.length,
                                        FieldConstants.width / 2.0,
                                        Rotation2d.fromDegrees(180))));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        //new AprilTagFieldLayout(tagList, FieldConstants.length, FieldConstants.width);
        AprilTagFieldLayout atfl = null;
        try {
                atfl = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
                e.printStackTrace();
        }

        // Forward Camera
        Timer timer = new Timer();
        System.out.println("Connecting to photon camera '" + VisionConstants.cameraName + "'");
        photonCamera =
                new PhotonCamera(
                        VisionConstants
                                .cameraName); // Change the name of your camera here to whatever it is in the
        System.out.println("Connected camera, " + photonCamera.getName() + ": " + photonCamera.isConnected());

         //Create pose estimator
         photonPoseEstimator =
                 new PhotonPoseEstimator(
                         atfl, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, VisionConstants.robotToCam);
                         */

    }

    public PhotonTrackedTarget getTarget() {
        return null;
        // PhotonPipelineResult result = photonCamera.getLatestResult();
        // if (result.hasTargets()) return result.getBestTarget();
        // else return null;
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        return Optional.empty();
        //  photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        //  return photonPoseEstimator.update();
    }
    public List<PhotonTrackedTarget> getTargets() {
        return new ArrayList<>();
        // var result = photonCamera.getLatestResult();
        // List<PhotonTrackedTarget> targets = result.getTargets();
        // return targets;
    }
}