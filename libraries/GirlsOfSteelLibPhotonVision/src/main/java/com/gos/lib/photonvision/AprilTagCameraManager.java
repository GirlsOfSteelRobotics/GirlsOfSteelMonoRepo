package com.gos.lib.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AprilTagCameraManager {
    private final List<AprilTagCamera> m_aprilTagCameras;

    private final VisionSystemSim m_visionSim;

    public AprilTagCameraManager(AprilTagFieldLayout tagLayout, List<AprilTagCamera> cameras) {
        m_aprilTagCameras = cameras;

        if (RobotBase.isSimulation()) {
            m_visionSim = new VisionSystemSim("AprilTagSim");
            for (AprilTagCamera camera : cameras) {
                m_visionSim.addCamera(camera.getSimulator(), camera.getRobotToCamera());
            }
            m_visionSim.addAprilTags(tagLayout);
        } else {
            m_visionSim = null;
        }
    }

    public List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> update(Pose2d prevEstimatedPose) {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> validEstimates = new ArrayList<>();

        for (AprilTagCamera camera : m_aprilTagCameras) {
            camera.update(prevEstimatedPose);
            Optional<EstimatedRobotPose> maybeResult = camera.getEstimateGlobalPose();

            if (maybeResult.isPresent()) {
                EstimatedRobotPose camPose = maybeResult.get();
                Pose2d camEstPose = camPose.estimatedPose.toPose2d();
                validEstimates.add(new Pair<>(camPose, camera.getEstimationStdDevs(camEstPose)));
            }
        }

        return validEstimates;
    }

    public void updateSimulator(Pose2d pose) {
        m_visionSim.update(pose);
    }

    public int numAprilTagsSeen() {
        int seen = 0;
        for (AprilTagCamera camera : m_aprilTagCameras) {
            Optional<PhotonPipelineResult> maybeResult = camera.getLatestResult();
            if (maybeResult.isPresent()) {
                seen += maybeResult.get().targets.size();
            }
        }
        return seen;
    }

    public void takeScreenshot() {
        for (AprilTagCamera camera : m_aprilTagCameras) {
            camera.takeScreenshot();
        }
    }

    public Pose3d getFirstEstimatedPose() {
        for (AprilTagCamera camera : m_aprilTagCameras) {
            Optional<EstimatedRobotPose> maybeResult = camera.getEstimateGlobalPose();
            if (maybeResult.isPresent()) {
                return maybeResult.get().estimatedPose;
            }
        }

        return null;
    }
}
