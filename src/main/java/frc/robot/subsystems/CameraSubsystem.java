// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
// import frc.robot.RobotContainer;

public class CameraSubsystem extends SubsystemBase {
    private static CameraSubsystem singleton = null;

    public static CameraSubsystem getSingleton() {
        if (singleton == null)
            singleton = new CameraSubsystem();
        return singleton;
    }

    private static final String limelightOneName = "limelight-fourone";
    private static final String limelightTwoName = "limelight-threeg";
    private static final boolean useLimelightTwo = true;

    private static final HashMap<Integer, AprilTag> aprilTagMap = new HashMap<>();
    private static final HashMap<Integer, PathPlannerPath> aprilTagLineUpMap = new HashMap<>();
    private static Translation2d reefCenterTranslation = new Translation2d();

    static boolean tryToMapAprilTagAndLineUp(int aprilTagID, String file_path) {
        try {
            aprilTagLineUpMap.put(aprilTagID, PathPlannerPath.fromPathFile(file_path));
            return true;
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to find path " + file_path, false);
        }
        return false;
    }
    static {
        for (int port = 5800; port <= 5809; port++)
            PortForwarder.add(port, limelightOneName + ".local", port);

        if (useLimelightTwo)
            for (int port = 5800; port <= 5809; port++)
                PortForwarder.add(port + 10, limelightTwoName + ".local", port);

        boolean success = false;
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            for (var tag : fieldLayout.getTags()) {
                aprilTagMap.put(tag.ID, tag);
            }
            success = true;
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), false);
        }

        if (success) {
            var translationAB = aprilTagMap.get(Constants.REEF_AB_TAGID).pose.getTranslation();
            var translationCD = aprilTagMap.get(Constants.REEF_CD_TAGID).pose.getTranslation();
            var translationEF = aprilTagMap.get(Constants.REEF_EF_TAGID).pose.getTranslation();
            var translationGH = aprilTagMap.get(Constants.REEF_GH_TAGID).pose.getTranslation();
            var translationIJ = aprilTagMap.get(Constants.REEF_IJ_TAGID).pose.getTranslation();
            var translationKL = aprilTagMap.get(Constants.REEF_KL_TAGID).pose.getTranslation();
            reefCenterTranslation = new Translation2d(
                (translationAB.getX() + translationCD.getX() + translationEF.getX() + translationGH.getX() + translationIJ.getX() + translationKL.getX()) / 6,
                (translationAB.getY() + translationCD.getY() + translationEF.getY() + translationGH.getY() + translationIJ.getY() + translationKL.getY()) / 6
            );

            success = success && tryToMapAprilTagAndLineUp(Constants.REEF_AB_TAGID, "REEF_AB_LINEUP")
                && tryToMapAprilTagAndLineUp(Constants.REEF_CD_TAGID, "REEF_CD_LINEUP")
                && tryToMapAprilTagAndLineUp(Constants.REEF_EF_TAGID, "REEF_EF_LINEUP")
                && tryToMapAprilTagAndLineUp(Constants.REEF_GH_TAGID, "REEF_GH_LINEUP")
                && tryToMapAprilTagAndLineUp(Constants.REEF_IJ_TAGID, "REEF_IJ_LINEUP")
                && tryToMapAprilTagAndLineUp(Constants.REEF_KL_TAGID, "REEF_KL_LINEUP");
        }
    }

    public static final class DynamicCommand extends Command {
        private final Supplier<Command> m_commandSupplier;
        private Command m_command;
        public DynamicCommand(Supplier<Command> commandSupplier) {
            m_commandSupplier = commandSupplier;
        }

        @Override
        public void initialize() {
            m_command = m_commandSupplier.get();
            m_command.schedule();
        }
        @Override
        public boolean isFinished() {
            return m_command.isFinished();
        }
        @Override
        public void end(boolean interuppted) {
            if (m_command.isScheduled())
                m_command.cancel();
        }
    }

    public static enum CoralStationID {
        Left(Constants.CORAL_STATION_LEFT_TAGID, Constants.CORAL_STATION_LEFT_OFFSET),
        Right(Constants.CORAL_STATION_RIGHT_TAGID, Constants.CORAL_STATION_RIGHT_OFFSET);

        // private final int m_tagID;
        private final Translation2d m_translation;
        private final Rotation2d m_rotation;
        private final Translation2d m_offset;

        CoralStationID(int tagID, Translation2d offset) {
            // m_tagID = tagID;
            final Pose2d pose = aprilTagMap.get(tagID).pose.toPose2d();
            m_translation = pose.getTranslation();
            m_rotation = pose.getRotation();
            m_offset = offset;
        }
    }

    public static enum RelativeReefLocation {
        AB(Constants.REEF_AB_TAGID),
        CD(Constants.REEF_CD_TAGID),
        EF(Constants.REEF_EF_TAGID),
        GH(Constants.REEF_GH_TAGID),
        IJ(Constants.REEF_IJ_TAGID),
        KL(Constants.REEF_KL_TAGID);

        private int m_tagID;
        private Translation2d m_translation;
        private Rotation2d m_rotation;
        private RelativeReefLocation m_next;
        private RelativeReefLocation m_previous;

        static {
            RelativeReefLocation first = AB;
            RelativeReefLocation previous = KL;
            RelativeReefLocation[] value_list = values();
            for (int index = 0; index < value_list.length; index++) {
                RelativeReefLocation value = value_list[index];
                int next_index = index + 1;
                value.m_next = next_index == value_list.length ? first : value_list[next_index];
                value.m_previous = previous;
                previous = value;
            }
        }

        RelativeReefLocation(int tagID) {
            m_tagID = tagID;
            final Pose2d pose = aprilTagMap.get(tagID).pose.toPose2d();
            m_translation = pose.getTranslation();
            m_rotation = pose.getRotation();
        }

        public int getTagID() {
            return m_tagID;
        }

        public RelativeReefLocation getNext() {
            return m_next;
        }
        public RelativeReefLocation getPrevious() {
            return m_previous;
        }
    }

    private static final double speedMultiplier = 2.5;
    private static final double rotatePID_P = 0.027;
    private static final double rangePID_P = 0.065;
    private static final double targetTagRange = -12.5;

    private static final boolean useMegaTag2 = true;
    private static final boolean tunePIDWithSmartDashboard = false;

    private Pose2d m_cachedPoseEstimate;

    private boolean m_insideReefZone = false;

    private final StructPublisher<Pose2d> swervePosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

    public CommandSwerveDrivetrain m_driveSubsystem;
    private Pigeon2 m_pigeon2;
    private double m_driveMaxSpeed;
    private double m_driveMaxAngularRate;
    public void setDriveSubsystem(CommandSwerveDrivetrain driveSubsystem, double driveMaxSpeed, double driveMaxAngularRate) {
        m_driveSubsystem = driveSubsystem;
        m_pigeon2 = m_driveSubsystem.getPigeon2();
        m_driveMaxSpeed = driveMaxSpeed;
        m_driveMaxAngularRate = driveMaxAngularRate;

        m_cachedPoseEstimate = m_driveSubsystem.getCustomEstimatedPose();
    }

    public CameraSubsystem() {
        if (tunePIDWithSmartDashboard) {
            SmartDashboard.putNumber("DRIVE_ROTATE_P", rotatePID_P);
            SmartDashboard.putNumber("DRIVE_RANGE_P", rangePID_P);
        }
    }

    public double calculateRotateFromTag() {
        double kP = rotatePID_P;

        if (tunePIDWithSmartDashboard) {
            kP = SmartDashboard.getNumber("DRIVE_ROTATE_P", rotatePID_P);
            if (kP > 0.5)
                kP = 0.5;
            else if (kP < -0.5)
                kP = -0.5;
        }

        double targetingAngularVelocity = LimelightHelpers.getTX(limelightOneName) * kP;
        targetingAngularVelocity *= m_driveMaxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    private final PIDController limelightRangeController = new PIDController(rangePID_P, 0, 0);
    public double calculateRangeFromTag() {    
        double kP = rangePID_P;

        if (tunePIDWithSmartDashboard) {
            kP = SmartDashboard.getNumber("DRIVE_RANGE_P", rangePID_P);
            if (kP > 0.5)
                kP = 0.5;
            else if (kP < -0.5)
                kP = -0.5;
        }

        limelightRangeController.setP(kP);
        double targetingForwardSpeed = limelightRangeController.calculate(LimelightHelpers.getTY(limelightOneName), targetTagRange);
        targetingForwardSpeed *= m_driveMaxSpeed * speedMultiplier;
        // targetingForwardSpeed *= -1.0;
        if (targetingForwardSpeed < 0)
            targetingForwardSpeed = 0;
        return targetingForwardSpeed;
    }

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.7,.7,9999999);
    private void updateVisionMegaTag1(String limelightName) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null)
            return;

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7)
                return;
            else if (mt1.rawFiducials[0].distToCamera > 3)
                return;
        } else if (mt1.tagCount == 0)
            return;

        Pose2d pose = mt1.pose;

        m_driveSubsystem.addCustomVisionMeasurement(
            pose, mt1.timestampSeconds,
            stdDevs
        );
    }
    private boolean updateVisionMegaTag2(String limelightName) {
        double yaw_degrees = m_cachedPoseEstimate.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(limelightName, yaw_degrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null)
            return false;

        if(Math.abs(m_pigeon2.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            return false;
        if(mt2.tagCount == 0)
            return false;

        Pose2d pose = mt2.pose;

        m_driveSubsystem.addCustomVisionMeasurement(
            pose, mt2.timestampSeconds,
            stdDevs
        );

        return true;
    }

    public void updateInsideReefZone(Translation2d robotTranslation) {
        // final RobotContainer robotContainer = RobotContainer.getSingleton();
        final double distance = robotTranslation.getDistance(reefCenterTranslation);
        SmartDashboard.putNumber("ReefZoneDistance", distance);
        final boolean insideReefZone = distance <= Constants.INSIDE_REEF_ZONE_THRESHOLD;
        if (insideReefZone != m_insideReefZone) {
            SmartDashboard.putBoolean("InsideReefZone", insideReefZone);
            // set field centric to be the opposite of if we're in the reef zone
            // robotContainer.setFieldCentric(m_insideReefZone); // optimization: m_insideReefZone should be the opposite
            m_insideReefZone = insideReefZone;
        }
    }

    @Override
    public void periodic() {
        m_cachedPoseEstimate = m_driveSubsystem.getCustomEstimatedPose();
        if (useMegaTag2) {
            SmartDashboard.putBoolean("MegaTag2SuccessOne", updateVisionMegaTag2(limelightOneName));
            if (useLimelightTwo) SmartDashboard.putBoolean("MegaTag2SuccessTwo", updateVisionMegaTag2(limelightTwoName));
        } else {
            updateVisionMegaTag1(limelightOneName);
            if (useLimelightTwo) updateVisionMegaTag1(limelightTwoName);
        }

        final Translation2d robotTranslation = m_cachedPoseEstimate.getTranslation();
        updateInsideReefZone(robotTranslation);

        swervePosePublisher.set(m_cachedPoseEstimate);
    }

    private final PIDController targetRotatePIDController = new PIDController(3, 0, 0);
    public double calculateRotateFromTag(int tagID) {
        Pose2d robotPose = m_cachedPoseEstimate;

        AprilTag targetTag = aprilTagMap.get(tagID);
        Pose2d targetTagPose = targetTag.pose.toPose2d();

        // double desiredAngle = Math.atan2(targetTagPose.getY() - robotPose.getY(), targetTagPose.getX() - robotPose.getX());
        // SmartDashboard.putNumber("ROTATEFROMTAG_DESIREDANGLE", desiredAngle);

        double desiredAngle = targetTagPose.getRotation().minus(robotPose.getRotation()).getRadians();
        SmartDashboard.putNumber("ROTATEFROMTAG_DESIREDANGLE", desiredAngle);

        double result = targetRotatePIDController.calculate(robotPose.getRotation().getRadians(), desiredAngle);
        SmartDashboard.putNumber("ROTATEFROMTAG_RESULT", result);
        return result;
    }

    public Command getPathCommandFromReefTag(RelativeReefLocation reefLocation) {
        final double offset = Units.inchesToMeters(25);

        // this code gets the target april tag position and applies a certain offset away from the reef
        final Translation2d targetTagTranslation = reefLocation.m_translation;
        Translation2d directionVector = targetTagTranslation.minus(reefCenterTranslation); // get vector from center of reef to tag
        final double directionVectorMagnitude = Math.sqrt(Math.pow(directionVector.getX(), 2) + Math.pow(directionVector.getY(), 2)); // get magnitude
        directionVector = new Translation2d(directionVector.getX() / directionVectorMagnitude, directionVector.getY() / directionVectorMagnitude); // normalize

        Pose2d targetPose = new Pose2d(
            new Translation2d(targetTagTranslation.getX() + directionVector.getX() * offset, targetTagTranslation.getY() + directionVector.getY() * offset),
            reefLocation.m_rotation.plus(Rotation2d.k180deg)
        );

        SmartDashboard.putNumber("PATHFINDING_POSEX", targetPose.getX());
        SmartDashboard.putNumber("PATHFINDING_POSEY", targetPose.getY());

        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        Command result = AutoBuilder.pathfindToPose(targetPose,
                constraints,
                0
        );

        // var lineup = aprilTagLineUpMap.getOrDefault(tagID, null);
        // if (lineup != null) {
        //     SmartDashboard.putString("LINING_UP", "" + System.currentTimeMillis());
        //     result = result.andThen(AutoBuilder.followPath(lineup));
        // }

        result.addRequirements(m_driveSubsystem);
        return result;
    }

    public Command getPathCommandFromCoralStationTag(CoralStationID coralStationID) {
        // this code gets the target april tag position and applies a certain offset away from the coral station
        final Translation2d targetTagTranslation = coralStationID.m_translation;

        final Translation2d offset = coralStationID.m_offset;

        Pose2d targetPose = new Pose2d(
            new Translation2d(targetTagTranslation.getX() + offset.getX(), targetTagTranslation.getY() + offset.getY()),
            coralStationID.m_rotation
        );

        SmartDashboard.putNumber("PATHFINDING_POSEX", targetPose.getX());
        SmartDashboard.putNumber("PATHFINDING_POSEY", targetPose.getY());

        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        Command result = AutoBuilder.pathfindToPose(targetPose,
                constraints,
                0
        );

        // var lineup = aprilTagLineUpMap.getOrDefault(tagID, null);
        // if (lineup != null) {
        //     SmartDashboard.putString("LINING_UP", "" + System.currentTimeMillis());
        //     result = result.andThen(AutoBuilder.followPath(lineup));
        // }

        result.addRequirements(m_driveSubsystem);
        return result;
    }
}