package org.frc5587.lib.pathfinder;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
// import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.frc5587.lib.TitanDrive;
import org.frc5587.lib.TitanDrive.DriveSignal;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SparkAbstractDrive extends Subsystem {
    protected CANSparkMax leftOne, rightOne;
    protected CANSparkMax leftTwo, rightTwo;
    protected CANPIDController spark_pidControllerLeft, spark_pidControllerRight;
    protected CANEncoder leftSparkEncoder, rightSparkEncoder;
    protected CANAnalog leftSparkAnalog, rightSparkAnalog;
    protected GyroBase gyro;
    protected AHRS ahrs;

    int minBufferCount;

    // convert to whatever unit sparks are in
    // private double maxVelocity = 2500; // Max velocity in STU (max value is 4096)
    private int timeoutMS = 10;
    // public int stuPerRev, stuPerInch, minBufferCount;
    public double wheelDiameterMeters;

    // private MotionProfileStatus[] statuses = { new MotionProfileStatus(), new MotionProfileStatus() };
    public Notifier profileNotifer = new Notifier(new ProcessProfileRunnable());

    public SparkAbstractDrive(CANSparkMax leftOne, CANSparkMax rightLeader, CANSparkMax leftFollower,
            CANSparkMax rightFollower, boolean flipRight) {
        // Set motors to correct objects and connect the masters and slaves
        this.leftOne = leftOne;
        this.leftTwo = leftFollower;
        this.rightOne = rightLeader;
        this.rightTwo = rightFollower;
        leftFollower.follow(leftOne);
        rightFollower.follow(rightLeader);

        // Left should not be reversed
        leftOne.setInverted(false);
        leftFollower.setInverted(false);

        // Invert the right side of the drivetrain
        if (flipRight) {
            rightLeader.setInverted(true);
            rightFollower.setInverted(true);
        } else {
            // The Talon might have been used in reversed state in past, so just to be sure
            rightLeader.setInverted(false);
            rightFollower.setInverted(false);
        }

        // Set encoder rotation as opposite direction relative to motor rotation
        // this.leftLeader.setSensorPhase(true);
        // this.rightMaster.setSensorPhase(true);

        leftSparkEncoder = leftOne.getEncoder();
        rightSparkEncoder = rightOne.getEncoder();

        this.ahrs = null;
        this.gyro = null;

        // configPID(0);
        configSettings();
        enableBrakeMode(true);
    }

    public void setGyro(GyroBase gyro) {
        this.gyro = gyro;
    }

    public void setAHRS(AHRS ahrs) {
        this.ahrs = ahrs;
    }

    public void setConstants(double maxVelocity, int timeoutMS, /*int stuPerRev, int stuPerInch,*/
            double wheelDiameterMeters, int minBufferCount) {
        // this.maxVelocity = maxVelocity;
        this.timeoutMS = timeoutMS;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.minBufferCount = minBufferCount;
    }

    public abstract void configPID(int slot);

    public abstract void configSettings();

    public void enableBrakeMode(boolean enabled) {
        if (enabled) {
            leftOne.setIdleMode(IdleMode.kBrake);
            rightOne.setIdleMode(IdleMode.kBrake);
            leftTwo.setIdleMode(IdleMode.kBrake);
            rightTwo.setIdleMode(IdleMode.kBrake);
        } else {
            leftOne.setIdleMode(IdleMode.kCoast);
            rightOne.setIdleMode(IdleMode.kCoast);
            leftTwo.setIdleMode(IdleMode.kCoast);
            rightTwo.setIdleMode(IdleMode.kCoast);
        }
    }

    /* --- BASIC MANUAL CONTROL CODE --- */

    public void vbusCurve(double throttle, double curve, boolean isQuickTurn) {
        DriveSignal d = TitanDrive.curvatureDrive(throttle, curve, isQuickTurn);

        leftOne.set(d.left);
        rightOne.set(d.right);
    }

    public void vbusArcade(double throttle, double turn) {
        DriveSignal d = TitanDrive.arcadeDrive(throttle, turn);

        leftOne.set(d.left);
        rightOne.set(d.right);
        leftTwo.set(d.left);
        rightTwo.set(d.right);
    }

    public void vbusLR(double left, double right) {
        leftOne.set(left);
        rightOne.set(right);
    }

    public void velocityCurve(double throttle, double curve, boolean isQuickTurn) {
        // DriveSignal d = TitanDrive.curvatureDrive(throttle, curve, isQuickTurn);

        // convert velocity to percent output?
        // leftMaster.set(ControlMode.Velocity, d.left * maxVelocity);
        // rightMaster.set(ControlMode.Velocity, d.right * maxVelocity);
    }

    public void velocityArcade(double throttle, double turn) {
        // DriveSignal d = TitanDrive.arcadeDrive(throttle, turn);

        // leftMaster.set(ControlMode.Velocity, d.left * maxVelocity);
        // rightMaster.set(ControlMode.Velocity, d.right * maxVelocity);
    }

    public void stop() {
        leftOne.disable();
        rightOne.disable();
        leftTwo.disable();
        rightTwo.disable();
    }

    /* --- MOTION PROFILE HANDLING CODE --- */

    public class ProcessProfileRunnable implements java.lang.Runnable {
        public void run() {
            // leftLeader.processMotionProfileBuffer();
            // rightLeader.processMotionProfileBuffer();
        }
    }

    public void resetMP() {
    //     leftMaster.clearMotionProfileHasUnderrun(timeoutMS);
    //     leftMaster.clearMotionProfileTrajectories();
    //     leftMaster.changeMotionControlFramePeriod(10);
    //     leftMaster.configMotionProfileTrajectoryPeriod(10, timeoutMS);
    //     leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutMS);
    //     leftMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);

    //     rightMaster.clearMotionProfileHasUnderrun(timeoutMS);
    //     rightMaster.clearMotionProfileTrajectories();
    //     rightMaster.changeMotionControlFramePeriod(10);
    //     rightMaster.configMotionProfileTrajectoryPeriod(10, timeoutMS);
    //     rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutMS);
    //     rightMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
    }

    public void updateStatus() {
        // leftMaster.getMotionProfileStatus(statuses[0]);
        // rightMaster.getMotionProfileStatus(statuses[1]);
    }

    // public boolean isMPReady() {
    //     // boolean leftReady = getStatuses()[0].btmBufferCnt > minBufferCount;
    //     // boolean rightReady = getStatuses()[0].btmBufferCnt > minBufferCount;
    //     return leftReady && rightReady;
    // }

    // public boolean isMPDone() {
    //     boolean leftDone = getStatuses()[0].isLast;
    //     boolean rightDone = getStatuses()[0].isLast;
    //     return leftDone && rightDone;
    // }

    // public void queuePoints(TrajectoryPoint[][] trajectories) {
        // for (TrajectoryPoint point : trajectories[0]) {
        //     leftMaster.pushMotionProfileTrajectory(point);
        // }
        // for (TrajectoryPoint point : trajectories[1]) {
        //     rightMaster.pushMotionProfileTrajectory(point);
        // }
    // }

    // public void setProfileMode(SetValueMotionProfile mpMode) {
        // leftMaster.set(ControlMode.MotionProfile, mpMode.value);
        // rightMaster.set(ControlMode.MotionProfile, mpMode.value);
    // }

    /* --- UTILITY METHODS --- */

    public void resetEncoders() {
        leftSparkEncoder.setPosition(0);
        rightSparkEncoder.setPosition(0);

        // ???
        leftSparkEncoder.setMeasurementPeriod(timeoutMS);
        rightSparkEncoder.setMeasurementPeriod(timeoutMS);
    }

    public void sendDebugInfo() {
        SmartDashboard.putNumber("Left Distance", getLeftPosition());
        SmartDashboard.putNumber("Right Distance", getRightPosition());
        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
        SmartDashboard.putNumber("Heading", getHeading());
    }

    public void sendMPDebugInfo() {
        SmartDashboard.putNumber("Left Pos", leftSparkEncoder.getPosition());
        SmartDashboard.putNumber("Right Pos", rightSparkEncoder.getPosition());
        SmartDashboard.putNumber("Left Vel", leftSparkEncoder.getVelocity());
        SmartDashboard.putNumber("Right Vel", rightSparkEncoder.getVelocity());
        // SmartDashboard.putNumber("Left Expected Pos", leftLeader.getActiveTrajectoryPosition());
        // SmartDashboard.putNumber("Right Expected Pos", rightMaster.getActiveTrajectoryPosition());
        // SmartDashboard.putNumber("Left Expected Vel", leftMaster.getActiveTrajectoryVelocity());
        // SmartDashboard.putNumber("Right Expected Vel", rightMaster.getActiveTrajectoryVelocity());
    }

    /* --- GETTER METHODS --- */

    public int getLeftPosition() {
        // returns in volts
        return (int)leftSparkEncoder.getPosition();
    }

    public int getRightPosition() {
        // return rightMaster.getSelectedSensorPosition(0);
        // returns in volts
        return (int)rightSparkEncoder.getPosition();

    }

    public int getLeftVelocity() {
        // return leftMaster.getSelectedSensorVelocity(0);
        // returns in volts per second
        return (int)leftSparkEncoder.getVelocity();
    }

    public int getRightVelocity() {
        // returns in volts per second
        return (int)rightSparkEncoder.getVelocity();
    }

    public double getLeftVoltage() {
        return leftSparkAnalog.getVoltage();
        // return leftMaster.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return rightSparkAnalog.getVoltage();
        // return rightMaster.getMotorOutputVoltage();
    }

    public double getHeading() {
        if (ahrs != null) {
            return ahrs.getAngle();
        } else if (gyro != null) {
            return gyro.getAngle();
        } else {
            System.out.println(
                    "Neither the AHRS nor a Gyro were set for the drivetrain before attempting to read heading");
            return Double.NaN;
        }
    }

    public double getHeading(Double wrapValue) {
        var heading = getHeading() % 360;
        return ((heading > 180.0) ? (heading - 360.0) : ((heading < -180.0) ? (heading + 360.0) : heading));
    }

    // public MotionProfileStatus[] getStatuses() {
    //     return statuses;
    // }
}