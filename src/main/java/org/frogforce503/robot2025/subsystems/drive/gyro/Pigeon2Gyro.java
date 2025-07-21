package org.frogforce503.robot2025.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro extends BaseGyro {

    PigeonIMU imu;

    // public Pigeon2Gyro (int deviceNumber, String canbus) {
    //     imu = new PigeonIMU(deviceNumber, canbus);
    // }

    public Pigeon2Gyro (int deviceNumber) {
        imu = new PigeonIMU(deviceNumber);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        imu.configFactoryDefault();
    }

    @Override
    public void setYaw(Rotation2d angle) {
        // TODO Auto-generated method stub
        imu.setYaw(angle.getDegrees());
    }

    @Override
    public Rotation2d getYaw() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(imu.getYaw());
    }

    @Override
    public Rotation2d getPitch() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(imu.getPitch());
    }

    @Override
    public Rotation2d getRoll() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(imu.getRoll());
    }
    
}