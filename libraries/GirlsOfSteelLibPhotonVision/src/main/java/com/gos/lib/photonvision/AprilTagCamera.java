package com.gos.lib.photonvision;

import com.gos.lib.field.AprilTagCameraObject;
import com.gos.lib.field.BaseGosField;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.TunableTransform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AprilTagCamera {
    public static final Matrix<N3, N1> DEFAULT_SINGLE_TAG_STDDEV = VecBuilder.fill(1.5, 1.5, 16); // (4, 4, 8)
    public static final Matrix<N3, N1> DEFAULT_MULTI_TAG_STDDEV = VecBuilder.fill(0.25, 0.25, 4); // (0.5, 0.5, 1)


    private final TunableTransform3d m_robotToCamera;
    private final String m_cameraName;
    private final Matrix<N3, N1> m_singleTagStddev;
    private final Matrix<N3, N1> m_multiTagStddev;


    private final PhotonCamera m_photonCamera;
    private final PhotonPoseEstimator m_photonPoseEstimator;
    private final PhotonCameraSim m_cameraSim;

    private final AprilTagCameraObject m_field;

    // Cached values
    private Optional<EstimatedRobotPose> m_maybeResult;
    private Optional<PhotonPipelineResult> m_lastPipelineResult;
    private int m_numTargetsSeen;
    private double m_avgDistanceToTag;
    private double m_avgAmbiguity;

    private final LoggingUtil m_logger;

    public AprilTagCamera(AprilTagFieldLayout aprilTagLayout, BaseGosField field, String name, TunableTransform3d transform3d) {
        this(aprilTagLayout, field, name, transform3d, DEFAULT_SINGLE_TAG_STDDEV, DEFAULT_MULTI_TAG_STDDEV);
    }


    public AprilTagCamera(AprilTagFieldLayout aprilTagLayout, BaseGosField field, String name, TunableTransform3d transform3d, Matrix<N3, N1> singleTagStddev, Matrix<N3, N1> multiTagStddev) {
        m_cameraName = name;
        m_robotToCamera = transform3d;
        m_photonCamera = new PhotonCamera(m_cameraName);
        m_singleTagStddev = singleTagStddev;
        m_multiTagStddev = multiTagStddev;
        m_field = new AprilTagCameraObject(field, m_cameraName);

        m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCamera.getTransform());
        m_photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        if (RobotBase.isSimulation()) {
            m_cameraSim = new PhotonCameraSim(m_photonCamera);

            boolean enableFancySim = true;
            m_cameraSim.enableRawStream(enableFancySim);
            m_cameraSim.enableProcessedStream(enableFancySim);
            m_cameraSim.enableDrawWireframe(enableFancySim);
        } else {
            m_cameraSim = null;
        }

        m_maybeResult = Optional.empty();

        m_logger = new LoggingUtil("GosCameras/" + m_cameraName);
        m_logger.addDouble("Targets Seen", () -> m_numTargetsSeen);
        m_logger.addDouble("Average Distance To Target", () -> m_avgDistanceToTag);
        m_logger.addDouble("Average Ambiguity", () -> m_avgAmbiguity);
    }

    public void update(Pose2d prevEstimatedRobotPose) {
        Transform3d robotToCamera = m_robotToCamera.getTransform();

        m_photonPoseEstimator.setRobotToCameraTransform(robotToCamera);
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        m_maybeResult = Optional.empty();
        m_lastPipelineResult = Optional.empty();

        for (PhotonPipelineResult result : m_photonCamera.getAllUnreadResults()) {
            update(robotToCamera, result);
        }
    }

    private void update(Transform3d robotToCamera, PhotonPipelineResult result) {
        m_maybeResult = m_photonPoseEstimator.update(result);

        if (m_maybeResult.isEmpty()) {
            m_field.clearCameraResult();
        } else {
            List<Pose3d> aprilTags = new ArrayList<>();
            EstimatedRobotPose estimatedRobotPose = m_maybeResult.get();
            for (PhotonTrackedTarget targetUsed : m_maybeResult.get().targetsUsed) {
                Pose3d bestTransformPosition =
                    estimatedRobotPose.estimatedPose
                        .transformBy(robotToCamera)
                        .transformBy(targetUsed.getBestCameraToTarget());
                aprilTags.add(bestTransformPosition);
            }
            m_field.setCameraResult(m_maybeResult.get().estimatedPose, aprilTags);
        }

        m_lastPipelineResult = Optional.of(result);

        m_logger.updateLogs();
    }

    public Optional<EstimatedRobotPose> getEstimateGlobalPose() {
        return m_maybeResult;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        Matrix<N3, N1> estStdDevs = m_singleTagStddev;
        if (m_lastPipelineResult.isEmpty()) {
            return estStdDevs;
        }
        List<PhotonTrackedTarget> targets = m_lastPipelineResult.get().getTargets();
        m_numTargetsSeen = 0;
        double sumDist = 0;
        double sumAmbiguity = 0;
        for (PhotonTrackedTarget tgt : targets) {
            Optional<Pose3d> tagPose = m_photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            m_numTargetsSeen++;
            sumDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
            sumAmbiguity += tgt.getPoseAmbiguity();
        }
        if (m_numTargetsSeen == 0) {
            return estStdDevs;
        }
        m_avgDistanceToTag = sumDist / m_numTargetsSeen;
        m_avgAmbiguity = sumAmbiguity / m_numTargetsSeen;
        // Decrease std devs if multiple targets are visible
        if (m_numTargetsSeen > 1) {
            estStdDevs = m_multiTagStddev;
        }
        // Increase std devs based on (average) distance
        if (m_numTargetsSeen == 1 && m_avgDistanceToTag > 3.2) {
            estStdDevs = VecBuilder.fill(1000, 1000, 1000);
        }
        else if (m_numTargetsSeen == 2 && m_avgDistanceToTag > 4) {
            estStdDevs = VecBuilder.fill(1000, 1000, 1000);
        }
        else {
            estStdDevs = estStdDevs.times(1 + (m_avgDistanceToTag * m_avgDistanceToTag / 30));
        }

        return estStdDevs;
    }

    public void takeScreenshot() {
        m_photonCamera.takeInputSnapshot();
        m_photonCamera.takeOutputSnapshot();
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        return m_lastPipelineResult;
    }

    public PhotonCameraSim getSimulator() {
        return m_cameraSim;
    }

    public Transform3d getRobotToCamera() {
        return m_robotToCamera.getTransform();
    }
}
