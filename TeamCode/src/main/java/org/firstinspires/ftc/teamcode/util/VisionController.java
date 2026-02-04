package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * Controller for vision processing using Limelight.
 * Handles distance calculation and recommended velocity for shooting.
 */
public class VisionController {

    private final Limelight3A limelight;
    private final IMU imu;

    private boolean canSeeTower = false;
    private boolean pointingToTower = false;
    private double distanceInches = 0;
    private double angleToTower = 0;
    private double recommendedVelocity = 0;

    public VisionController(RobotHardware hardware) {
        this.limelight = hardware.limelight;
        this.imu = hardware.imu;
    }

    /**
     * Start the Limelight vision system.
     */
    public void start() {
        if (limelight != null) {
            limelight.start();
            limelight.pipelineSwitch(0);
        }
    }

    /**
     * Update vision data. Call this in the loop.
     */
    public void update() {
        if (limelight == null || imu == null) {
            canSeeTower = false;
            return;
        }

        // Update Limelight with robot orientation
        limelight.updateRobotOrientation(
                imu.getRobotYawPitchRollAngles().getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Check freshness
            canSeeTower = result.getStaleness() < RobotConstants.VISION_STALENESS_THRESHOLD;

            // Get angles
            double tx = result.getTx();
            double ty = result.getTy();
            angleToTower = tx;

            // Calculate distance
            double angleToTagDeg = RobotConstants.CAMERA_PITCH_DEG + ty;
            double angleToTagRad = Math.toRadians(angleToTagDeg);
            distanceInches = (RobotConstants.TAG_HEIGHT_IN - RobotConstants.CAMERA_HEIGHT_IN)
                    / Math.tan(angleToTagRad);

            // Calculate recommended velocity
            recommendedVelocity = (RobotConstants.MAX_SHOOTER_VELOCITY / 100.0)
                    * calcRecommendedVelocityPct(distanceInches);

            // Check if pointing at tower
            pointingToTower = Math.abs(tx) < RobotConstants.TOWER_ANGLE_TOLERANCE;
        } else {
            canSeeTower = false;
        }
    }

    /**
     * Calculate recommended velocity percentage based on distance.
     */
    private double calcRecommendedVelocityPct(double distance) {
        double ret = RobotConstants.VELOCITY_SLOPE * distance + RobotConstants.VELOCITY_INTERCEPT;
        ret = Math.max(ret, 0);
        ret = Math.min(ret, 100);
        return ret;
    }

    /**
     * Check if the tower is visible.
     */
    public boolean canSeeTower() {
        return canSeeTower;
    }

    /**
     * Check if the robot is pointing at the tower.
     */
    public boolean isPointingToTower() {
        return pointingToTower;
    }

    /**
     * Get the distance to the tower in inches.
     */
    public double getDistanceInches() {
        return distanceInches;
    }

    /**
     * Get the angle to the tower (horizontal offset).
     */
    public double getAngleToTower() {
        return angleToTower;
    }

    /**
     * Get the recommended velocity for shooting.
     */
    public double getRecommendedVelocity() {
        return recommendedVelocity;
    }

    /**
     * Get the horizontal offset from Limelight (tx).
     */
    public double getTx() {
        if (limelight == null) return 0;
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0;
    }

    /**
     * Get the vertical offset from Limelight (ty).
     */
    public double getTy() {
        if (limelight == null) return 0;
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0;
    }

    /**
     * Get the target area from Limelight (ta).
     */
    public double getTa() {
        if (limelight == null) return 0;
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTa() : 0;
    }
}
