package com.robotstudio.controller.model;

/**
 * 文件信息数据模型
 */
public class FileInfo {
    private String name;
    private long size;
    private String modified;
    private String type;
    private int fileCount;  // 对于点云目录，表示包含的文件数量
    private String downloadUrl;
    
    public FileInfo() {
        this.name = "";
        this.size = 0;
        this.modified = "";
        this.type = "";
        this.fileCount = 0;
        this.downloadUrl = "";
    }
    
    /**
     * 格式化文件大小
     */
    public String getFormattedSize() {
        if (size < 1024) {
            return size + " B";
        } else if (size < 1024 * 1024) {
            return String.format("%.1f KB", size / 1024.0);
        } else if (size < 1024 * 1024 * 1024) {
            return String.format("%.1f MB", size / (1024.0 * 1024.0));
        } else {
            return String.format("%.1f GB", size / (1024.0 * 1024.0 * 1024.0));
        }
    }
    
    /**
     * 获取文件扩展名
     */
    public String getExtension() {
        int lastDot = name.lastIndexOf('.');
        if (lastDot > 0 && lastDot < name.length() - 1) {
            return name.substring(lastDot + 1).toLowerCase();
        }
        return "";
    }
    
    /**
     * 检查是否为地图文件
     */
    public boolean isMapFile() {
        return "maps".equals(type) || "yaml".equals(getExtension()) || "pgm".equals(getExtension());
    }
    
    /**
     * 检查是否为点云文件
     */
    public boolean isPointCloudFile() {
        return "clouds".equals(type) || "ply".equals(getExtension()) || "xyz".equals(getExtension());
    }
    
    // Getters and Setters
    public String getName() {
        return name;
    }
    
    public void setName(String name) {
        this.name = name;
    }
    
    public long getSize() {
        return size;
    }
    
    public void setSize(long size) {
        this.size = size;
    }
    
    public String getModified() {
        return modified;
    }
    
    public void setModified(String modified) {
        this.modified = modified;
    }
    
    public String getType() {
        return type;
    }
    
    public void setType(String type) {
        this.type = type;
    }
    
    public int getFileCount() {
        return fileCount;
    }
    
    public void setFileCount(int fileCount) {
        this.fileCount = fileCount;
    }
    
    public String getDownloadUrl() {
        return downloadUrl;
    }
    
    public void setDownloadUrl(String downloadUrl) {
        this.downloadUrl = downloadUrl;
    }
    
    @Override
    public String toString() {
        return "FileInfo{" +
                "name='" + name + '\'' +
                ", size=" + size +
                ", modified='" + modified + '\'' +
                ", type='" + type + '\'' +
                ", fileCount=" + fileCount +
                '}';
    }
}
