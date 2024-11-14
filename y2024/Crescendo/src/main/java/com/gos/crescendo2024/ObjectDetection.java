package com.gos.crescendo2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

import static com.gos.crescendo2024.FieldConstants.StagingLocations.CENTERLINE_TRANSLATIONS;
import static com.gos.crescendo2024.FieldConstants.StagingLocations.SPIKE_TRANSLATIONS;

public class ObjectDetection {
    private static final Transform3d ROBOT_TO_CAMERA = RobotExtrinsics.ROBOT_TO_CAM_OBJECT_DETECTION;

    private final PhotonCamera m_photonCamera;

    private final VisionSystemSim m_visionSim;
    private final PhotonCameraSim m_cameraSim;

    public ObjectDetection() {
        m_photonCamera = new PhotonCamera("USB_Camera");

        TargetModel targetModel = new TargetModel(Units.inchesToMeters(18));

        if (RobotBase.isSimulation()) {
            m_cameraSim = new PhotonCameraSim(m_photonCamera);

            boolean enableFancySim = false;
            m_cameraSim.enableRawStream(enableFancySim);
            m_cameraSim.enableProcessedStream(enableFancySim);
            m_cameraSim.enableDrawWireframe(enableFancySim);


            m_visionSim = new VisionSystemSim("ObjectDetection");
            m_visionSim.addCamera(m_cameraSim, ROBOT_TO_CAMERA);

            for (Translation2d centerlineTranslation : CENTERLINE_TRANSLATIONS) {
                Pose3d targetPose = new Pose3d(new Translation3d(centerlineTranslation.getX(), centerlineTranslation.getY(), 0), new Rotation3d());
                VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
                m_visionSim.addVisionTargets(visionTarget);
            }

            for (Translation2d spikeTranslation : SPIKE_TRANSLATIONS) {
                Pose3d targetPose = new Pose3d(new Translation3d(spikeTranslation.getX(), spikeTranslation.getY(), 0), new Rotation3d());
                VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
                m_visionSim.addVisionTargets(visionTarget);
            }
        } else {
            m_cameraSim = null;
            m_visionSim = null;
        }
    }

    public void updateObjectDetectionSimulation(Pose2d chassisLocation) {
        m_visionSim.update(chassisLocation);
    }

    public static double calculateDistanceToTarget(
        double cameraHeightMeters,
        double targetHeightMeters,
        double cameraPitchRadians,
        double targetPitchRadians,
        double targetYawRadians) {
        return (targetHeightMeters - cameraHeightMeters)
            / (Math.tan(cameraPitchRadians + targetPitchRadians) * Math.cos(targetYawRadians));
    }

    public List<Pose2d> objectLocations(Pose2d chassisLocation) {
        List<Pose2d> objectLocationsList = new ArrayList<>();
        List<PhotonPipelineResult> results = m_photonCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return objectLocationsList;
        }

        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        for (PhotonTrackedTarget result : latestResult.getTargets()) {
            Rotation2d yaw = Rotation2d.fromDegrees(-result.getYaw());
            double distance = calculateDistanceToTarget(
                ROBOT_TO_CAMERA.getZ(),
                0,
                -ROBOT_TO_CAMERA.getRotation().getY(),
                Units.degreesToRadians(result.getPitch()),
                yaw.getRadians()
            );
            Translation2d relToCamera = PhotonUtils.estimateCameraToTargetTranslation(distance, yaw);
            relToCamera = new Translation2d(
                relToCamera.getX() + ROBOT_TO_CAMERA.getX(),
                relToCamera.getY() + ROBOT_TO_CAMERA.getY());
            Transform2d adjustForRot = new Transform2d(relToCamera, new Rotation2d());
            objectLocationsList.add(chassisLocation.transformBy(adjustForRot));
        }
        return objectLocationsList;
    }
}
