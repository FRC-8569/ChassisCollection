package org.ZenithPolaris.Collections;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ZenithPolaris.Collections.Wheels.SwerveWheels.SwerveModules;
import org.ZenithPolaris.Collections.utils.DifferentialWheels;
import org.ZenithPolaris.Collections.utils.GyroInterface;
import org.ZenithPolaris.Collections.utils.KOPChassisConfig;
import org.ZenithPolaris.Collections.utils.MecanumChassisConfig;
import org.ZenithPolaris.Collections.utils.SwerveChassisConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class Chassis {

    public static class KOPChassis{
        public DifferentialWheels<?>[] LeftMotor, RightMotor;
        public GyroInterface<?> gyro;
        public KOPChassisConfig chassisConfig;
        public DifferentialDriveKinematics kinematics;
        public DifferentialDrivePoseEstimator poseEstimator;
        public StructPublisher<Pose2d> PosePublish = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).publish();
        public BooleanPublisher TempAlert = NetworkTableInstance.getDefault().getBooleanTopic("Drivetrain/isTempLimitHit").publish();

        /**
         * 
         * @param wheels is the differential drive wheel, it accepted two or four motor counts, which is (L,R) or (L1,L2,R1,R2)
         */
        public KOPChassis(KOPChassisConfig chassisConfig,DifferentialWheels<?>... wheels){
            if(wheels.length == 2){
                LeftMotor[0] = wheels[0];
                RightMotor[0] = wheels[1];
            }else if(wheels.length == 4){
                LeftMotor[0] = wheels[0];
                LeftMotor[1] = wheels[1];
                RightMotor[0] = wheels[2];
                RightMotor[1] = wheels[3];
            }else throw new Error("Wheel count not available");
            this.chassisConfig = chassisConfig;
            this.kinematics = new DifferentialDriveKinematics(chassisConfig.TrackWidth);
        }

        /**
         * 
         * @param gyro is the gyroscope of your robot, which offers odometry and PoseEstmator feature
         * @param wheels
         */
        public KOPChassis(GyroInterface<?> gyro,KOPChassisConfig chassisConfig, DifferentialWheels<?>... wheels){
            if(wheels.length == 2){
                LeftMotor[0] = wheels[0];
                RightMotor[0] = wheels[1];
            }else if(wheels.length == 4){
                LeftMotor[0] = wheels[0];
                LeftMotor[1] = wheels[1];
                RightMotor[0] = wheels[2];
                RightMotor[1] = wheels[3];
            }else throw new Error("wheel count not available");

            this.chassisConfig = chassisConfig;
            this.gyro = gyro;
            this.poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), getPosition().leftMeters, getPosition().rightMeters, new Pose2d(0,0,gyro.getRotation2d()));
            this.kinematics = new DifferentialDriveKinematics(chassisConfig.TrackWidth);
        }
        
        public DifferentialDriveWheelPositions getPosition(){
            return new DifferentialDriveWheelPositions(
                LeftMotor[0].getPosition(), 
                RightMotor[0].getPosition());
        }

        public DifferentialDriveWheelSpeeds getVelocity(){
            return new DifferentialDriveWheelSpeeds(
                LeftMotor[0].getVelocity(),
                RightMotor[0].getVelocity()
            );
        }

        public void drive(ChassisSpeeds speeds){
            DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds.discretize(speeds, 0.02));
            wheelSpeeds.desaturate(chassisConfig.maxVelocity);
            setWheelSpeeds(wheelSpeeds);
        }

        public void drive(double speed, double rotation){
            DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(chassisConfig.maxVelocity.times(speed+rotation), chassisConfig.maxVelocity.times(speed-rotation));
            speeds.desaturate(chassisConfig.maxVelocity);
            setWheelSpeeds(speeds);
        }

        private void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds){
            for(var motor : LeftMotor){
                motor.setVelocity(MetersPerSecond.of(speeds.leftMetersPerSecond));
            }

            for(var motor : RightMotor){
                motor.setVelocity(MetersPerSecond.of(speeds.rightMetersPerSecond));
            }
        }


        /**
         * this is the function for robot information update, put it in periodic of your code
         */
        public void telemrtry(){
            PosePublish.accept(poseEstimator.update(gyro.getRotation2d(), getPosition()));
            TempAlert.accept(LeftMotor[0].getMotorTemp().gte(chassisConfig.TempLimit) || RightMotor[0].getMotorTemp().gte(chassisConfig.TempLimit));
        }
    }

    public static class MecanumChassis{
        public DifferentialWheels<?> FrontLeft, FrontRight, BackLeft, BackRight;
        public GyroInterface<?> gyro;
        public MecanumDriveKinematics kinematics;
        public MecanumDriveOdometry odometry;
        public MecanumChassisConfig chassisConfig;

        public MecanumChassis(GyroInterface<?> gyro, MecanumChassisConfig chassisConfig, DifferentialWheels<?>... wheels){
            if(wheels.length != 4) throw new Error("Wheel count is not available");
            FrontLeft = wheels[0];
            FrontRight = wheels[1];
            BackLeft = wheels[2];
            BackRight = wheels[3];
            this.gyro = gyro;
            this.chassisConfig = chassisConfig;
            kinematics = new MecanumDriveKinematics(chassisConfig.WheelPlaces[0], chassisConfig.WheelPlaces[1], chassisConfig.WheelPlaces[2], chassisConfig.WheelPlaces[3]);
            odometry = new MecanumDriveOdometry(kinematics,gyro.getRotation2d() , getPositions());
        }

        public MecanumDriveWheelPositions getPositions(){
            return new MecanumDriveWheelPositions(
                FrontLeft.getPosition(),
                FrontRight.getPosition(),
                BackLeft.getPosition(),
                BackRight.getPosition()
            );
        }

        public ChassisSpeeds getVelocity(){
            return kinematics.toChassisSpeeds(getSpeeds());
        }

        private MecanumDriveWheelSpeeds getSpeeds(){
            return new MecanumDriveWheelSpeeds(
                FrontLeft.getVelocity(),
                FrontRight.getVelocity(),
                BackLeft.getVelocity(),
                BackRight.getVelocity()
            );
        }

        public void drive(ChassisSpeeds speeds){
            setSpeed(kinematics.toWheelSpeeds(ChassisSpeeds.discretize(speeds, 0.02)));
        }

        public void setSpeed(MecanumDriveWheelSpeeds speeds){
            speeds.desaturate(chassisConfig.MaxSpeed);
            FrontLeft.setVelocity(MetersPerSecond.of(speeds.frontLeftMetersPerSecond));
            BackLeft.setVelocity(MetersPerSecond.of(speeds.rearLeftMetersPerSecond));
            FrontRight.setVelocity(MetersPerSecond.of(speeds.frontRightMetersPerSecond));
            BackRight.setVelocity(MetersPerSecond.of(speeds.rearRightMetersPerSecond));
        }
    }

    public static class SwerveChassis {
        public SwerveModules[] modules;
        public SwerveDriveKinematics kinematics;
        public SwerveDriveOdometry odometry;
        public GyroInterface<?> gyro;
        /**
         * 
         * @param config
         * @param modules needs four modules, FL,FR,BL,BR
         */
        public SwerveChassis(SwerveChassisConfig config, GyroInterface<?> gyro, SwerveModules... modules){
            if(modules.length != 4) throw new Error("The count of module is not avalialbe");
            this.modules = modules;
            this.gyro = gyro;
            kinematics = new SwerveDriveKinematics(config.places);
            odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d() , getPositions());
        }

        public SwerveModulePosition[] getPositions(){
            return new SwerveModulePosition[]{
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()
            };
        }

        public SwerveModuleState[] getStates(){
            return new SwerveModuleState[]{
                modules[0].getState(),
                modules[1].getState(),
                modules[2].getState(),
                modules[3].getState()
            };
        }

        public void drive(ChassisSpeeds speeds){
            setStates(   
                kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02))
            );
        }

        public void setStates(SwerveModuleState[] states){
            modules[0].setState(states[0]);
            modules[1].setState(states[1]);
            modules[2].setState(states[2]);
            modules[0].setState(states[3]);
        }
    }
}
