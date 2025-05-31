package org.ZenithPolars.ChassisCollection;

import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Gyroscope;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.KOPChassisConfig;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.MecanumChassisConfig;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.NormalDriveWheels;
import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Swerve.SwerveChassisConfig;
import org.ZenithPolars.ChassisCollection.Wheels.Swerve.SwerveModule;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Chassis {
    public class KOPChassis {
        public NormalDriveWheels<?>[] LeftWheels, RightWheels;
        public Gyroscope<?> gyro;
        public DifferentialDriveKinematics kinematics;
        public DifferentialDrivePoseEstimator poseEstimator;
        public Field2d field2d;
        public KOPChassisConfig chassisConfig;

        /**
         * 
         * @param gyro          is the gyroscope of the chassis, which is
         *                      {@link Gyro.NavXGyro} and {@link Gyro.Pigeon2Gyro}
         * @param chassisConfig is the chassis configruation
         * @param wheels        is the wheels of chassis, accepts two or four wheels, 2
         *                      for L,R, 4 for LF,RF,LB,RB
         * @param InitialPose   is the initial pose of your robot, you can use new
         *                      Pose2d(0,0) for easy setup
         */
        public KOPChassis(Gyroscope<?> gyro, KOPChassisConfig chassisConfig, Pose2d InitialPose,
                NormalDriveWheels<?>... wheels) {
            if (wheels.length == 2) {
                LeftWheels[0] = wheels[0];
                RightWheels[0] = wheels[1];
            } else if (wheels.length == 4) {
                LeftWheels[0] = wheels[0];
                LeftWheels[1] = wheels[2];
                RightWheels[0] = wheels[1];
                RightWheels[1] = wheels[3];
            }

            this.gyro = gyro;
            kinematics = new DifferentialDriveKinematics(chassisConfig.TrackWidth);
            poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(),
                    getPostitions().leftMeters, getPostitions().rightMeters, InitialPose);
            this.chassisConfig = chassisConfig;
            field2d = new Field2d();
        }

        public DifferentialDriveWheelPositions getPostitions() {
            return new DifferentialDriveWheelPositions(
                    LeftWheels[0].getPosition(),
                    RightWheels[0].getPosition());
        }

        public DifferentialDriveWheelSpeeds getSpeeds() {
            return new DifferentialDriveWheelSpeeds(
                    LeftWheels[0].getVelocity(),
                    RightWheels[0].getVelocity());
        }

        public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
            speeds.desaturate(chassisConfig.MaxSpeed);
            for (var wheel : LeftWheels) {
                wheel.setSpeedMPS(speeds.leftMetersPerSecond);
            }

            for (var wheel : RightWheels) {
                wheel.setSpeedMPS(speeds.rightMetersPerSecond);
            }
        }

        public void telemerize() {
            field2d.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getPostitions()));
            SmartDashboard.putData(field2d);
        }
    }

    public class MecanumChassis {
        public NormalDriveWheels<?>[] wheels;
        public Gyroscope<?> gyro;
        public MecanumChassisConfig chassisConfig;
        public MecanumDriveKinematics kinematics;
        public MecanumDrivePoseEstimator poseEstimator;
        public Field2d field2d;

        public MecanumChassis(Gyroscope<?> gyro, MecanumChassisConfig chassisConfig, Pose2d InitialPose,
                NormalDriveWheels<?>... ChassisWheels) {
            if (wheels.length != 4)
                throw new Error("Wheel count invalid");

            this.wheels = ChassisWheels;
            this.gyro = gyro;
            kinematics = new MecanumDriveKinematics(chassisConfig.WheelPlaces[0], chassisConfig.WheelPlaces[1],
                    chassisConfig.WheelPlaces[2], chassisConfig.WheelPlaces[3]);
            poseEstimator = new MecanumDrivePoseEstimator(kinematics, gyro.getRotation2d(), getPosition(), InitialPose);
            this.chassisConfig = chassisConfig;
        }

        public MecanumDriveWheelSpeeds getVelocity() {
            return new MecanumDriveWheelSpeeds(
                    wheels[0].getVelocity(),
                    wheels[1].getVelocity(),
                    wheels[2].getVelocity(),
                    wheels[3].getVelocity());
        }

        public MecanumDriveWheelPositions getPosition() {
            return new MecanumDriveWheelPositions(
                    wheels[0].getPosition(),
                    wheels[1].getPosition(),
                    wheels[2].getPosition(),
                    wheels[3].getPosition());
        }

        public void drive(ChassisSpeeds speeds) {
            kinematics.toWheelSpeeds(ChassisSpeeds.discretize(speeds, 0.02)).desaturate(chassisConfig.MaxVelocity);
        }

        public void telemerize() {
            field2d.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getPosition()));
            SmartDashboard.putData(field2d);
        }
    }

    public class SwerveChassis {
        public SwerveModule[] modules;
        public Gyroscope<?> gyro;
        public SwerveDriveKinematics kinematics;
        public SwerveDriveOdometry odometry;
        public StructArrayPublisher<SwerveModuleState> TargetState, CurrentState;
        public StructPublisher<ChassisSpeeds> ChassisSpeedsPublisher; 

        /**
         * 
         * @param chassisConfig is the chassis configuration
         * @param modules is the swerve modules which needs four.
         */
        public SwerveChassis(Gyroscope<?> gyro ,SwerveChassisConfig chassisConfig, SwerveModule... modules){
            this.modules = modules;
            this.gyro = gyro;
            kinematics = new SwerveDriveKinematics(chassisConfig.WheelPlaces);
            odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPosition());
            TargetState = NetworkTableInstance.getDefault().getStructArrayTopic("Drivetrain/TargetState", SwerveModuleState.struct).publish();
            CurrentState = NetworkTableInstance.getDefault().getStructArrayTopic("Drivetrain/CurrentState", SwerveModuleState.struct).publish();
            ChassisSpeedsPublisher = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/ChassiSpeeds", ChassisSpeeds.struct).publish();
        }

        public SwerveModulePosition[] getPosition() {
            return new SwerveModulePosition[] {
                    modules[0].getPosition(),
                    modules[1].getPosition(),
                    modules[2].getPosition(),
                    modules[3].getPosition()
            };
        }

        public SwerveModuleState[] getState(){
            return new SwerveModuleState[]{
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()
            };
        }
    
        public void drive(ChassisSpeeds speeds){
            setStates(kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)));
            ChassisSpeedsPublisher.accept(speeds);
        }

        public void setStates(SwerveModuleState... states){
            for(int i = 0; i < states.length; i++){
                modules[i].setState(states[i]);
            }
            TargetState.accept(states);
        }

        public void telemerize(){
            odometry.update(gyro.getRotation2d(), getPosition());
            CurrentState.accept(getState());
        }
    }
}