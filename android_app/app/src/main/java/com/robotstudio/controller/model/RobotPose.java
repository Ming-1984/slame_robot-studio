package com.robotstudio.controller.model;

/**
 * 机器人位姿数据模型
 */
public class RobotPose {
    // 位置
    private double x;
    private double y;
    private double z;
    
    // 四元数方向
    private double qx;
    private double qy;
    private double qz;
    private double qw;
    
    public RobotPose() {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
        this.qx = 0.0;
        this.qy = 0.0;
        this.qz = 0.0;
        this.qw = 1.0;
    }
    
    public RobotPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.qx = qx;
        this.qy = qy;
        this.qz = qz;
        this.qw = qw;
    }
    
    // 计算欧拉角（弧度）
    public double getYaw() {
        return Math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
    
    // 计算欧拉角（度）
    public double getYawDegrees() {
        return Math.toDegrees(getYaw());
    }
    
    // Getters and Setters
    public double getX() {
        return x;
    }
    
    public void setX(double x) {
        this.x = x;
    }
    
    public double getY() {
        return y;
    }
    
    public void setY(double y) {
        this.y = y;
    }
    
    public double getZ() {
        return z;
    }
    
    public void setZ(double z) {
        this.z = z;
    }
    
    public double getQx() {
        return qx;
    }
    
    public void setQx(double qx) {
        this.qx = qx;
    }
    
    public double getQy() {
        return qy;
    }
    
    public void setQy(double qy) {
        this.qy = qy;
    }
    
    public double getQz() {
        return qz;
    }
    
    public void setQz(double qz) {
        this.qz = qz;
    }
    
    public double getQw() {
        return qw;
    }
    
    public void setQw(double qw) {
        this.qw = qw;
    }
    
    @Override
    public String toString() {
        return String.format("RobotPose{pos=(%.2f, %.2f, %.2f), yaw=%.1f°}", 
                x, y, z, getYawDegrees());
    }
}
