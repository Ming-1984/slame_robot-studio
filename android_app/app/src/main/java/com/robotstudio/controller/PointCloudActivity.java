package com.robotstudio.controller;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ZoomControls;

import androidx.appcompat.app.AppCompatActivity;

import com.robotstudio.controller.network.ApiClient;
import com.robotstudio.controller.network.ApiCallback;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

/**
 * 点云显示活动
 */
public class PointCloudActivity extends AppCompatActivity {
    
    private ApiClient apiClient;
    
    // UI组件
    private ImageView imageViewPointCloud;
    private Button btnRefresh, btnSavePointCloud, btnLoadPointCloud, btnStartProcessing;
    private ProgressBar progressBar;
    private TextView tvStatus, tvPointCloudInfo;
    private ZoomControls zoomControls;
    
    // 点云数据
    private String currentPointCloudName = "";
    private Bitmap currentPointCloudBitmap;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_point_cloud);
        
        initViews();
        setupControls();
        
        // 初始化API客户端
        apiClient = new ApiClient();
        String serverIp = MainActivity.getServerIp(this);
        apiClient.setServerAddress(serverIp);
        
        // 检查点云处理状态
        checkPointCloudStatus();
    }
    
    private void initViews() {
        imageViewPointCloud = findViewById(R.id.image_view_point_cloud);
        btnRefresh = findViewById(R.id.btn_refresh);
        btnSavePointCloud = findViewById(R.id.btn_save_point_cloud);
        btnLoadPointCloud = findViewById(R.id.btn_load_point_cloud);
        btnStartProcessing = findViewById(R.id.btn_start_processing);
        progressBar = findViewById(R.id.progress_bar);
        tvStatus = findViewById(R.id.tv_status);
        tvPointCloudInfo = findViewById(R.id.tv_point_cloud_info);
        zoomControls = findViewById(R.id.zoom_controls);
    }
    
    private void setupControls() {
        btnRefresh.setOnClickListener(v -> loadCurrentPointCloud());
        
        btnSavePointCloud.setOnClickListener(v -> {
            if (currentPointCloudBitmap != null) {
                savePointCloudToGallery();
            } else {
                Toast.makeText(this, "没有可保存的点云图像", Toast.LENGTH_SHORT).show();
            }
        });
        
        btnLoadPointCloud.setOnClickListener(v -> showPointCloudSelectionDialog());
        
        btnStartProcessing.setOnClickListener(v -> startPointCloudProcessing());
        
        // 缩放控制
        zoomControls.setOnZoomInClickListener(v -> {
            imageViewPointCloud.setScaleX(imageViewPointCloud.getScaleX() * 1.2f);
            imageViewPointCloud.setScaleY(imageViewPointCloud.getScaleY() * 1.2f);
        });
        
        zoomControls.setOnZoomOutClickListener(v -> {
            imageViewPointCloud.setScaleX(imageViewPointCloud.getScaleX() / 1.2f);
            imageViewPointCloud.setScaleY(imageViewPointCloud.getScaleY() / 1.2f);
        });
    }
    
    private void checkPointCloudStatus() {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("检查点云处理状态...");
        
        apiClient.checkPointCloudStatus(new ApiCallback<String>() {
            @Override
            public void onSuccess(String status) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("点云状态: " + status);
                    
                    if ("ready".equals(status) || "available".equals(status)) {
                        loadCurrentPointCloud();
                    } else {
                        btnStartProcessing.setEnabled(true);
                        Toast.makeText(PointCloudActivity.this, "点云数据未就绪，请先开始处理", Toast.LENGTH_SHORT).show();
                    }
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("状态检查失败: " + error);
                    btnStartProcessing.setEnabled(true);
                });
            }
        });
    }
    
    private void startPointCloudProcessing() {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("正在启动点云处理...");
        btnStartProcessing.setEnabled(false);
        
        apiClient.executeSystemCommand(
            com.robotstudio.controller.model.SystemCommand.startPointCloudProcessing(),
            new ApiCallback<String>() {
                @Override
                public void onSuccess(String result) {
                    runOnUiThread(() -> {
                        progressBar.setVisibility(View.GONE);
                        tvStatus.setText("点云处理已启动");
                        Toast.makeText(PointCloudActivity.this, "点云处理已启动，请稍候...", Toast.LENGTH_LONG).show();
                        
                        // 延迟检查状态
                        new android.os.Handler().postDelayed(() -> {
                            checkPointCloudStatus();
                        }, 5000);
                    });
                }
                
                @Override
                public void onError(String error) {
                    runOnUiThread(() -> {
                        progressBar.setVisibility(View.GONE);
                        tvStatus.setText("启动失败: " + error);
                        btnStartProcessing.setEnabled(true);
                        Toast.makeText(PointCloudActivity.this, "启动点云处理失败: " + error, Toast.LENGTH_LONG).show();
                    });
                }
            }
        );
    }
    
    private void loadCurrentPointCloud() {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("正在加载点云图像...");
        
        // 获取当前点云图像
        apiClient.getCurrentPointCloud(new ApiCallback<String>() {
            @Override
            public void onSuccess(String pointCloudUrl) {
                runOnUiThread(() -> {
                    loadPointCloudImage(pointCloudUrl);
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("加载失败: " + error);
                    Toast.makeText(PointCloudActivity.this, "加载点云图像失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void loadPointCloudImage(String pointCloudUrl) {
        new Thread(() -> {
            try {
                URL url = new URL(pointCloudUrl);
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(10000);
                connection.setReadTimeout(30000);
                
                if (connection.getResponseCode() == HttpURLConnection.HTTP_OK) {
                    InputStream inputStream = connection.getInputStream();
                    Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                    inputStream.close();
                    
                    runOnUiThread(() -> {
                        progressBar.setVisibility(View.GONE);
                        currentPointCloudBitmap = bitmap;
                        imageViewPointCloud.setImageBitmap(bitmap);
                        tvStatus.setText("点云图像加载完成");
                        tvPointCloudInfo.setText("图像尺寸: " + bitmap.getWidth() + "x" + bitmap.getHeight());
                    });
                } else {
                    runOnUiThread(() -> {
                        progressBar.setVisibility(View.GONE);
                        tvStatus.setText("加载失败: HTTP " + connection.getResponseCode());
                    });
                }
                
            } catch (IOException e) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("加载失败: " + e.getMessage());
                });
            }
        }).start();
    }
    
    private void showPointCloudSelectionDialog() {
        // 获取可用点云列表
        apiClient.getPointCloudList(new ApiCallback<java.util.List<String>>() {
            @Override
            public void onSuccess(java.util.List<String> pointCloudList) {
                runOnUiThread(() -> {
                    if (pointCloudList.isEmpty()) {
                        Toast.makeText(PointCloudActivity.this, "没有可用的点云数据", Toast.LENGTH_SHORT).show();
                        return;
                    }
                    
                    String[] pointClouds = pointCloudList.toArray(new String[0]);
                    new android.app.AlertDialog.Builder(PointCloudActivity.this)
                        .setTitle("选择点云")
                        .setItems(pointClouds, (dialog, which) -> {
                            currentPointCloudName = pointClouds[which];
                            loadSelectedPointCloud(currentPointCloudName);
                        })
                        .setNegativeButton("取消", null)
                        .show();
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(PointCloudActivity.this, "获取点云列表失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void loadSelectedPointCloud(String pointCloudName) {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("正在加载点云: " + pointCloudName);
        
        apiClient.getPointCloudImage(pointCloudName, new ApiCallback<String>() {
            @Override
            public void onSuccess(String pointCloudUrl) {
                runOnUiThread(() -> {
                    loadPointCloudImage(pointCloudUrl);
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("加载失败: " + error);
                    Toast.makeText(PointCloudActivity.this, "加载点云失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void savePointCloudToGallery() {
        try {
            String fileName = "pointcloud_" + System.currentTimeMillis() + ".png";
            java.io.File file = new java.io.File(getExternalFilesDir(null), fileName);
            java.io.FileOutputStream out = new java.io.FileOutputStream(file);
            currentPointCloudBitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.flush();
            out.close();
            
            Toast.makeText(this, "点云图像已保存到: " + file.getAbsolutePath(), Toast.LENGTH_LONG).show();
            
        } catch (Exception e) {
            Toast.makeText(this, "保存失败: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }
} 