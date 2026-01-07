package com.robotstudio.controller.model;

import java.util.HashMap;
import java.util.Map;

/**
 * 系统命令数据模型
 */
public class SystemCommand {
    private String command;
    private Map<String, Object> parameters;
    
    public SystemCommand(String command) {
        this.command = command;
        this.parameters = new HashMap<>();
    }
    
    public SystemCommand(String command, Map<String, Object> parameters) {
        this.command = command;
        this.parameters = parameters != null ? parameters : new HashMap<>();
    }
    
    /**
     * 添加参数
     */
    public SystemCommand addParameter(String key, Object value) {
        parameters.put(key, value);
        return this;
    }
    
    // 预定义的系统命令
    public static SystemCommand shutdown() {
        return new SystemCommand("shutdown_system").addParameter("confirm", "关机");
    }
    
    public static SystemCommand startPathPlanning() {
        return new SystemCommand("start_path_planning");
    }
    
    public static SystemCommand stopPathPlanning() {
        return new SystemCommand("stop_path_planning");
    }
    
    public static SystemCommand emergencyStop() {
        return new SystemCommand("emergency_stop");
    }
    
    public static SystemCommand saveMap() {
        return new SystemCommand("save_map");
    }
    
    public static SystemCommand startPointCloudProcessing() {
        return new SystemCommand("start_point_cloud_processing");
    }
    
    public static SystemCommand savePointCloud() {
        return new SystemCommand("save_point_cloud");
    }
    
    public static SystemCommand checkPointCloudStatus() {
        return new SystemCommand("check_point_cloud_status");
    }
    
    // Getters and Setters
    public String getCommand() {
        return command;
    }
    
    public void setCommand(String command) {
        this.command = command;
    }
    
    public Map<String, Object> getParameters() {
        return parameters;
    }
    
    public void setParameters(Map<String, Object> parameters) {
        this.parameters = parameters;
    }
    
    @Override
    public String toString() {
        return "SystemCommand{" +
                "command='" + command + '\'' +
                ", parameters=" + parameters +
                '}';
    }
}
