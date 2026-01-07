package com.robotstudio.controller.model;

/**
 * 系统状态数据模型
 */
public class SystemStatus {
    private boolean connected;
    private int batteryLevel;
    private String systemTime;
    private String rosStatus;
    private String wifiStatus;
    private String navigationStatus;
    
    public SystemStatus() {
        this.connected = false;
        this.batteryLevel = 0;
        this.systemTime = "";
        this.rosStatus = "unknown";
        this.wifiStatus = "unknown";
        this.navigationStatus = "unknown";
    }
    
    // Getters and Setters
    public boolean isConnected() {
        return connected;
    }
    
    public void setConnected(boolean connected) {
        this.connected = connected;
    }
    
    public int getBatteryLevel() {
        return batteryLevel;
    }
    
    public void setBatteryLevel(int batteryLevel) {
        this.batteryLevel = batteryLevel;
    }
    
    public String getSystemTime() {
        return systemTime;
    }
    
    public void setSystemTime(String systemTime) {
        this.systemTime = systemTime;
    }
    
    public String getRosStatus() {
        return rosStatus;
    }
    
    public void setRosStatus(String rosStatus) {
        this.rosStatus = rosStatus;
    }
    
    public String getWifiStatus() {
        return wifiStatus;
    }
    
    public void setWifiStatus(String wifiStatus) {
        this.wifiStatus = wifiStatus;
    }
    
    public String getNavigationStatus() {
        return navigationStatus;
    }
    
    public void setNavigationStatus(String navigationStatus) {
        this.navigationStatus = navigationStatus;
    }
    
    @Override
    public String toString() {
        return "SystemStatus{" +
                "connected=" + connected +
                ", batteryLevel=" + batteryLevel +
                ", systemTime='" + systemTime + '\'' +
                ", rosStatus='" + rosStatus + '\'' +
                ", wifiStatus='" + wifiStatus + '\'' +
                ", navigationStatus='" + navigationStatus + '\'' +
                '}';
    }
}
