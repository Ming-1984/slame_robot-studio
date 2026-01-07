package com.robotstudio.controller.network;

import android.util.Log;
import com.robotstudio.controller.model.SystemStatus;
import com.robotstudio.controller.model.RobotPose;
import com.robotstudio.controller.model.VelocityCommand;
import com.robotstudio.controller.model.SystemCommand;
import com.robotstudio.controller.model.FileInfo;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

/**
 * API客户端 - 与机器人服务器通信
 */
public class ApiClient {
    private static final String TAG = "ApiClient";
    private static final MediaType JSON = MediaType.get("application/json; charset=utf-8");
    
    private String serverAddress = "192.168.4.1";
    private int apiPort = 8000;
    private String baseUrl;
    
    private OkHttpClient httpClient;
    
    public ApiClient() {
        // 配置HTTP客户端
        httpClient = new OkHttpClient.Builder()
                .connectTimeout(10, TimeUnit.SECONDS)
                .readTimeout(30, TimeUnit.SECONDS)
                .writeTimeout(30, TimeUnit.SECONDS)
                .retryOnConnectionFailure(true)
                .build();
        
        updateBaseUrl();
    }
    
    /**
     * 设置服务器地址
     */
    public void setServerAddress(String address) {
        this.serverAddress = address;
        updateBaseUrl();
    }
    
    private void updateBaseUrl() {
        this.baseUrl = "http://" + serverAddress + ":" + apiPort + "/api";
        Log.d(TAG, "API Base URL: " + baseUrl);
    }
    
    /**
     * 获取系统状态
     */
    public void getSystemStatus(ApiCallback<SystemStatus> callback) {
        String url = baseUrl + "/status";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取系统状态失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        SystemStatus status = parseSystemStatus(responseBody);
                        callback.onSuccess(status);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析系统状态失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取机器人位姿
     */
    public void getRobotPose(ApiCallback<RobotPose> callback) {
        String url = baseUrl + "/robot/pose";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取机器人位姿失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        RobotPose pose = parseRobotPose(responseBody);
                        callback.onSuccess(pose);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析机器人位姿失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 发送速度控制命令
     */
    public void sendVelocityCommand(VelocityCommand command, ApiCallback<String> callback) {
        String url = baseUrl + "/robot/velocity";
        
        try {
            JSONObject json = new JSONObject();
            json.put("linear_x", command.getLinearX());
            json.put("linear_y", command.getLinearY());
            json.put("angular_z", command.getAngularZ());
            
            RequestBody body = RequestBody.create(json.toString(), JSON);
            Request request = new Request.Builder()
                    .url(url)
                    .post(body)
                    .build();
            
            httpClient.newCall(request).enqueue(new Callback() {
                @Override
                public void onFailure(Call call, IOException e) {
                    Log.e(TAG, "发送速度命令失败", e);
                    callback.onError("网络连接失败: " + e.getMessage());
                }
                
                @Override
                public void onResponse(Call call, Response response) throws IOException {
                    try {
                        if (response.isSuccessful()) {
                            callback.onSuccess("速度命令发送成功");
                        } else {
                            callback.onError("服务器错误: " + response.code());
                        }
                    } finally {
                        response.close();
                    }
                }
            });
            
        } catch (JSONException e) {
            Log.e(TAG, "创建速度命令JSON失败", e);
            callback.onError("命令格式错误: " + e.getMessage());
        }
    }
    
    /**
     * 执行系统命令
     */
    public void executeSystemCommand(SystemCommand command, ApiCallback<String> callback) {
        String url = baseUrl + "/system/command";
        
        try {
            JSONObject json = new JSONObject();
            json.put("command", command.getCommand());
            if (command.getParameters() != null) {
                json.put("parameters", new JSONObject(command.getParameters()));
            }
            
            RequestBody body = RequestBody.create(json.toString(), JSON);
            Request request = new Request.Builder()
                    .url(url)
                    .post(body)
                    .build();
            
            httpClient.newCall(request).enqueue(new Callback() {
                @Override
                public void onFailure(Call call, IOException e) {
                    Log.e(TAG, "执行系统命令失败", e);
                    callback.onError("网络连接失败: " + e.getMessage());
                }
                
                @Override
                public void onResponse(Call call, Response response) throws IOException {
                    try {
                        if (response.isSuccessful()) {
                            String responseBody = response.body().string();
                            callback.onSuccess(responseBody);
                        } else {
                            callback.onError("服务器错误: " + response.code());
                        }
                    } finally {
                        response.close();
                    }
                }
            });
            
        } catch (JSONException e) {
            Log.e(TAG, "创建系统命令JSON失败", e);
            callback.onError("命令格式错误: " + e.getMessage());
        }
    }
    
    /**
     * 获取文件列表
     */
    public void getFileList(String fileType, ApiCallback<List<FileInfo>> callback) {
        String url = baseUrl + "/files/list/" + fileType;
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取文件列表失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        List<FileInfo> files = parseFileList(responseBody, fileType);
                        callback.onSuccess(files);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析文件列表失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取下载URL
     */
    public String getDownloadUrl(String fileType, String fileName) {
        if ("maps".equals(fileType)) {
            return baseUrl + "/download/map/" + fileName;
        } else if ("clouds".equals(fileType)) {
            return baseUrl + "/download/cloud/" + fileName;
        } else {
            return baseUrl + "/files/download/" + fileName;
        }
    }
    
    /**
     * 获取当前地图URL
     */
    public void getCurrentMap(ApiCallback<String> callback) {
        String url = baseUrl + "/map/current";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取当前地图失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        String mapUrl = obj.optString("map_url", "");
                        callback.onSuccess(mapUrl);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析当前地图失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取地图列表
     */
    public void getMapList(ApiCallback<List<String>> callback) {
        String url = baseUrl + "/map/list";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取地图列表失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        JSONArray mapsArray = obj.getJSONArray("maps");
                        List<String> maps = new ArrayList<>();
                        
                        for (int i = 0; i < mapsArray.length(); i++) {
                            maps.add(mapsArray.getString(i));
                        }
                        
                        callback.onSuccess(maps);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析地图列表失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取指定地图图像URL
     */
    public void getMapImage(String mapName, ApiCallback<String> callback) {
        String url = baseUrl + "/map/image/" + mapName;
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取地图图像失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        String imageUrl = obj.optString("image_url", "");
                        callback.onSuccess(imageUrl);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析地图图像失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取当前点云URL
     */
    public void getCurrentPointCloud(ApiCallback<String> callback) {
        String url = baseUrl + "/pointcloud/current";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取当前点云失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        String pointCloudUrl = obj.optString("pointcloud_url", "");
                        callback.onSuccess(pointCloudUrl);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析当前点云失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取点云列表
     */
    public void getPointCloudList(ApiCallback<List<String>> callback) {
        String url = baseUrl + "/pointcloud/list";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取点云列表失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        JSONArray pointCloudsArray = obj.getJSONArray("pointclouds");
                        List<String> pointClouds = new ArrayList<>();
                        
                        for (int i = 0; i < pointCloudsArray.length(); i++) {
                            pointClouds.add(pointCloudsArray.getString(i));
                        }
                        
                        callback.onSuccess(pointClouds);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析点云列表失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 获取指定点云图像URL
     */
    public void getPointCloudImage(String pointCloudName, ApiCallback<String> callback) {
        String url = baseUrl + "/pointcloud/image/" + pointCloudName;
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "获取点云图像失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        String imageUrl = obj.optString("image_url", "");
                        callback.onSuccess(imageUrl);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析点云图像失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    /**
     * 检查点云处理状态
     */
    public void checkPointCloudStatus(ApiCallback<String> callback) {
        String url = baseUrl + "/pointcloud/status";
        
        Request request = new Request.Builder()
                .url(url)
                .get()
                .build();
        
        httpClient.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                Log.e(TAG, "检查点云状态失败", e);
                callback.onError("网络连接失败: " + e.getMessage());
            }
            
            @Override
            public void onResponse(Call call, Response response) throws IOException {
                try {
                    if (response.isSuccessful()) {
                        String responseBody = response.body().string();
                        JSONObject obj = new JSONObject(responseBody);
                        String status = obj.optString("status", "unknown");
                        callback.onSuccess(status);
                    } else {
                        callback.onError("服务器错误: " + response.code());
                    }
                } catch (Exception e) {
                    Log.e(TAG, "解析点云状态失败", e);
                    callback.onError("数据解析失败: " + e.getMessage());
                } finally {
                    response.close();
                }
            }
        });
    }
    
    // 解析方法
    private SystemStatus parseSystemStatus(String json) throws JSONException {
        JSONObject obj = new JSONObject(json);
        SystemStatus status = new SystemStatus();
        
        status.setConnected(obj.optBoolean("connected", false));
        status.setBatteryLevel(obj.optInt("battery_level", 0));
        status.setSystemTime(obj.optString("system_time", ""));
        status.setRosStatus(obj.optString("ros_status", "unknown"));
        
        return status;
    }
    
    private RobotPose parseRobotPose(String json) throws JSONException {
        JSONObject obj = new JSONObject(json);
        RobotPose pose = new RobotPose();
        
        if (obj.has("position")) {
            JSONObject pos = obj.getJSONObject("position");
            pose.setX(pos.optDouble("x", 0.0));
            pose.setY(pos.optDouble("y", 0.0));
            pose.setZ(pos.optDouble("z", 0.0));
        }
        
        if (obj.has("orientation")) {
            JSONObject ori = obj.getJSONObject("orientation");
            pose.setQx(ori.optDouble("x", 0.0));
            pose.setQy(ori.optDouble("y", 0.0));
            pose.setQz(ori.optDouble("z", 0.0));
            pose.setQw(ori.optDouble("w", 1.0));
        }
        
        return pose;
    }
    
    private List<FileInfo> parseFileList(String json, String fileType) throws JSONException {
        JSONObject obj = new JSONObject(json);
        JSONArray filesArray = obj.getJSONArray("files");
        List<FileInfo> files = new ArrayList<>();
        
        for (int i = 0; i < filesArray.length(); i++) {
            JSONObject fileObj = filesArray.getJSONObject(i);
            FileInfo fileInfo = new FileInfo();
            
            fileInfo.setName(fileObj.optString("name", ""));
            fileInfo.setSize(fileObj.optLong("total_size", fileObj.optLong("yaml_size", 0) + fileObj.optLong("pgm_size", 0)));
            fileInfo.setModified(fileObj.optString("modified", ""));
            fileInfo.setType(fileType);
            
            if ("clouds".equals(fileType)) {
                fileInfo.setFileCount(fileObj.optInt("file_count", 0));
            }
            
            files.add(fileInfo);
        }
        
        return files;
    }
}
