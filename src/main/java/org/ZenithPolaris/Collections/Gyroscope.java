package org.ZenithPolaris.Collections;

import org.ZenithPolaris.Collections.utils.GyroInterface;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscope {
    public class NavXGyro implements GyroInterface<AHRS> {
        public AHRS gyro;
        
        public NavXGyro(NavXComType com){
            gyro = new AHRS(com);
        }
        
        @Override
        public Rotation2d getRotation2d(){
            return gyro.getRotation2d();
        }

        @Override
        public AHRS getGyro(){
            return gyro;
        }
    }

    public class Pigeon2Gyro implements GyroInterface<Pigeon2> {
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
