package com.robotstudio.controller.model;

/**
 * 速度控制命令数据模型
 */
public class VelocityCommand {
    private double linearX;   // 前进/后退速度 (m/s)
    private double linearY;   // 左右平移速度 (m/s)
    private double angularZ;  // 旋转速度 (rad/s)
    
    public VelocityCommand() {
        this.linearX = 0.0;
        this.linearY = 0.0;
        this.angularZ = 0.0;
    }
    
    public VelocityCommand(double linearX, double linearY, double angularZ) {
        this.linearX = linearX;
        this.linearY = linearY;
        this.angularZ = angularZ;
    }
    
    /**
     * 创建停止命令
     */
    public static VelocityCommand stop() {
        return new VelocityCommand(0.0, 0.0, 0.0);
    }
    
    /**
     * 创建前进命令
     */
    public static VelocityCommand forward(double speed) {
        return new VelocityCommand(speed, 0.0, 0.0);
    }
    
    /**
     * 创建后退命令
     */
    public static VelocityCommand backward(double speed) {
        return new VelocityCommand(-speed, 0.0, 0.0);
    }
    
    /**
     * 创建左转命令
     */
    public static VelocityCommand turnLeft(double angularSpeed) {
        return new VelocityCommand(0.0, 0.0, angularSpeed);
    }
    
    /**
     * 创建右转命令
     */
    public static VelocityCommand turnRight(double angularSpeed) {
        return new VelocityCommand(0.0, 0.0, -angularSpeed);
    }
    
    /**
     * 检查是否为停止命令
     */
    public boolean isStop() {
        return linearX == 0.0 && linearY == 0.0 && angularZ == 0.0;
    }
    
    /**
     * 限制速度范围
     */
    public VelocityCommand clamp(double maxLinear, double maxAngular) {
        double clampedLinearX = Math.max(-maxLinear, Math.min(maxLinear, linearX));
        double clampedLinearY = Math.max(-maxLinear, Math.min(maxLinear, linearY));
        double clampedAngularZ = Math.max(-maxAngular, Math.min(maxAngular, angularZ));
        
        return new VelocityCommand(clampedLinearX, clampedLinearY, clampedAngularZ);
    }
    
    // Getters and Setters
    public double getLinearX() {
        return linearX;
    }
    
    public void setLinearX(double linearX) {
        this.linearX = linearX;
    }
    
    public double getLinearY() {
        return linearY;
    }
    
    public void setLinearY(double linearY) {
        this.linearY = linearY;
    }
    
    public double getAngularZ() {
        return angularZ;
    }
    
    public void setAngularZ(double angularZ) {
        this.angularZ = angularZ;
    }
    
    @Override
    public String toString() {
        return String.format("VelocityCommand{linear=(%.2f, %.2f), angular=%.2f}", 
                linearX, linearY, angularZ);
    }
}
