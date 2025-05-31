package org.ZenithPolars.ChassisCollection;

import org.ZenithPolars.ChassisCollection.utils.DifferentialDrives.Gyroscope;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {
    public class NavXGyro implements Gyroscope<AHRS>{
        public AHRS gyro;

        public NavXGyro(NavXComType type){
            gyro = new AHRS(type);
        }

        @Override
        public Rotation2d getRotation2d(){
            return this.getRotation2d();
        }

        @Override
        public AHRS getGyro(){
            return gyro;
        }
    }

    public class Pigeon2Gyro implements Gyroscope<Pigeon2>{
        public Pigeon2 gyro;
        
        public Pigeon2Gyro(int CANID){
            gyro = new Pigeon2(CANID);
        }

        @Override
        public Rotation2d getRotation2d(){
            return gyro.getRotation2d();
        }

        @Override
        public Pigeon2 getGyro(){
            return gyro;
        }
    }
}
