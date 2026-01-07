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
 * 地图显示活动
 */
public class MapDisplayActivity extends AppCompatActivity {
    
    private ApiClient apiClient;
    
    // UI组件
    private ImageView imageViewMap;
    private Button btnRefresh, btnSaveMap, btnLoadMap;
    private ProgressBar progressBar;
    private TextView tvStatus, tvMapInfo;
    private ZoomControls zoomControls;
    
    // 地图数据
    private String currentMapName = "";
    private Bitmap currentMapBitmap;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_map_display);
        
        initViews();
        setupControls();
        
        // 初始化API客户端
        apiClient = new ApiClient();
        String serverIp = MainActivity.getServerIp(this);
        apiClient.setServerAddress(serverIp);
        
        // 加载默认地图
        loadCurrentMap();
    }
    
    private void initViews() {
        imageViewMap = findViewById(R.id.image_view_map);
        btnRefresh = findViewById(R.id.btn_refresh);
        btnSaveMap = findViewById(R.id.btn_save_map);
        btnLoadMap = findViewById(R.id.btn_load_map);
        progressBar = findViewById(R.id.progress_bar);
        tvStatus = findViewById(R.id.tv_status);
        tvMapInfo = findViewById(R.id.tv_map_info);
        zoomControls = findViewById(R.id.zoom_controls);
    }
    
    private void setupControls() {
        btnRefresh.setOnClickListener(v -> loadCurrentMap());
        
        btnSaveMap.setOnClickListener(v -> {
            if (currentMapBitmap != null) {
                saveMapToGallery();
            } else {
                Toast.makeText(this, "没有可保存的地图", Toast.LENGTH_SHORT).show();
            }
        });
        
        btnLoadMap.setOnClickListener(v -> showMapSelectionDialog());
        
        // 缩放控制
        zoomControls.setOnZoomInClickListener(v -> {
            imageViewMap.setScaleX(imageViewMap.getScaleX() * 1.2f);
            imageViewMap.setScaleY(imageViewMap.getScaleY() * 1.2f);
        });
        
        zoomControls.setOnZoomOutClickListener(v -> {
            imageViewMap.setScaleX(imageViewMap.getScaleX() / 1.2f);
            imageViewMap.setScaleY(imageViewMap.getScaleY() / 1.2f);
        });
    }
    
    private void loadCurrentMap() {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("正在加载地图...");
        
        // 获取当前地图
        apiClient.getCurrentMap(new ApiCallback<String>() {
            @Override
            public void onSuccess(String mapUrl) {
                runOnUiThread(() -> {
                    loadMapImage(mapUrl);
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("加载失败: " + error);
                    Toast.makeText(MapDisplayActivity.this, "加载地图失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void loadMapImage(String mapUrl) {
        new Thread(() -> {
            try {
                URL url = new URL(mapUrl);
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(10000);
                connection.setReadTimeout(30000);
                
                if (connection.getResponseCode() == HttpURLConnection.HTTP_OK) {
                    InputStream inputStream = connection.getInputStream();
                    Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                    inputStream.close();
                    
                    runOnUiThread(() -> {
                        progressBar.setVisibility(View.GONE);
                        currentMapBitmap = bitmap;
                        imageViewMap.setImageBitmap(bitmap);
                        tvStatus.setText("地图加载完成");
                        tvMapInfo.setText("地图尺寸: " + bitmap.getWidth() + "x" + bitmap.getHeight());
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
    
    private void showMapSelectionDialog() {
        // 获取可用地图列表
        apiClient.getMapList(new ApiCallback<java.util.List<String>>() {
            @Override
            public void onSuccess(java.util.List<String> mapList) {
                runOnUiThread(() -> {
                    if (mapList.isEmpty()) {
                        Toast.makeText(MapDisplayActivity.this, "没有可用的地图", Toast.LENGTH_SHORT).show();
                        return;
                    }
                    
                    String[] maps = mapList.toArray(new String[0]);
                    new android.app.AlertDialog.Builder(MapDisplayActivity.this)
                        .setTitle("选择地图")
                        .setItems(maps, (dialog, which) -> {
                            currentMapName = maps[which];
                            loadSelectedMap(currentMapName);
                        })
                        .setNegativeButton("取消", null)
                        .show();
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(MapDisplayActivity.this, "获取地图列表失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void loadSelectedMap(String mapName) {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("正在加载地图: " + mapName);
        
        apiClient.getMapImage(mapName, new ApiCallback<String>() {
            @Override
            public void onSuccess(String mapUrl) {
                runOnUiThread(() -> {
                    loadMapImage(mapUrl);
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("加载失败: " + error);
                    Toast.makeText(MapDisplayActivity.this, "加载地图失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void saveMapToGallery() {
        try {
            String fileName = "map_" + System.currentTimeMillis() + ".png";
            java.io.File file = new java.io.File(getExternalFilesDir(null), fileName);
            java.io.FileOutputStream out = new java.io.FileOutputStream(file);
            currentMapBitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.flush();
            out.close();
            
            Toast.makeText(this, "地图已保存到: " + file.getAbsolutePath(), Toast.LENGTH_LONG).show();
            
        } catch (Exception e) {
            Toast.makeText(this, "保存失败: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }
} 