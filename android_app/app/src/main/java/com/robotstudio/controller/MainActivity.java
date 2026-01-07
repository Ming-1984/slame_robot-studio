package com.robotstudio.controller;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.cardview.widget.CardView;

import com.robotstudio.controller.network.ApiClient;
import com.robotstudio.controller.network.ApiCallback;
import com.robotstudio.controller.model.SystemStatus;

/**
 * 主活动 - 应用入口和连接设置
 */
public class MainActivity extends AppCompatActivity {
    
    private static final String PREFS_NAME = "RobotStudioPrefs";
    private static final String PREF_SERVER_IP = "server_ip";
    private static final String DEFAULT_SERVER_IP = "192.168.4.1";
    
    private EditText etServerIp;
    private Button btnConnect;
    private TextView tvConnectionStatus;
    private CardView cardRobotControl;
    private CardView cardMapDisplay;
    private CardView cardPointCloud;
    private CardView cardFileManager;
    private CardView cardSettings;
    
    private ApiClient apiClient;
    private boolean isConnected = false;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        initViews();
        loadSettings();
        setupClickListeners();
        
        // 初始化API客户端
        apiClient = new ApiClient();
    }
    
    private void initViews() {
        etServerIp = findViewById(R.id.et_server_ip);
        btnConnect = findViewById(R.id.btn_connect);
        tvConnectionStatus = findViewById(R.id.tv_connection_status);
        cardRobotControl = findViewById(R.id.card_robot_control);
        cardMapDisplay = findViewById(R.id.card_map_display);
        cardPointCloud = findViewById(R.id.card_point_cloud);
        cardFileManager = findViewById(R.id.card_file_manager);
        cardSettings = findViewById(R.id.card_settings);
        
        // 初始状态下禁用功能卡片
        setFunctionCardsEnabled(false);
    }
    
    private void loadSettings() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        String serverIp = prefs.getString(PREF_SERVER_IP, DEFAULT_SERVER_IP);
        etServerIp.setText(serverIp);
    }
    
    private void saveSettings() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putString(PREF_SERVER_IP, etServerIp.getText().toString().trim());
        editor.apply();
    }
    
    private void setupClickListeners() {
        btnConnect.setOnClickListener(v -> {
            if (isConnected) {
                disconnect();
            } else {
                connect();
            }
        });
        
        cardRobotControl.setOnClickListener(v -> {
            if (isConnected) {
                startActivity(new Intent(this, RobotControlActivity.class));
            } else {
                showNotConnectedMessage();
            }
        });
        
        cardMapDisplay.setOnClickListener(v -> {
            if (isConnected) {
                startActivity(new Intent(this, MapDisplayActivity.class));
            } else {
                showNotConnectedMessage();
            }
        });
        
        cardPointCloud.setOnClickListener(v -> {
            if (isConnected) {
                startActivity(new Intent(this, PointCloudActivity.class));
            } else {
                showNotConnectedMessage();
            }
        });
        
        cardFileManager.setOnClickListener(v -> {
            if (isConnected) {
                startActivity(new Intent(this, FileManagerActivity.class));
            } else {
                showNotConnectedMessage();
            }
        });
        
        cardSettings.setOnClickListener(v -> {
            startActivity(new Intent(this, SettingsActivity.class));
        });
    }
    
    private void connect() {
        String serverIp = etServerIp.getText().toString().trim();
        
        if (serverIp.isEmpty()) {
            Toast.makeText(this, "请输入服务器IP地址", Toast.LENGTH_SHORT).show();
            return;
        }
        
        btnConnect.setEnabled(false);
        btnConnect.setText("连接中...");
        tvConnectionStatus.setText("正在连接...");
        
        // 保存设置
        saveSettings();
        
        // 设置服务器地址
        apiClient.setServerAddress(serverIp);
        
        // 测试连接
        apiClient.getSystemStatus(new ApiCallback<SystemStatus>() {
            @Override
            public void onSuccess(SystemStatus result) {
                runOnUiThread(() -> {
                    isConnected = true;
                    btnConnect.setEnabled(true);
                    btnConnect.setText("断开连接");
                    tvConnectionStatus.setText("已连接到 " + serverIp);
                    setFunctionCardsEnabled(true);
                    
                    Toast.makeText(MainActivity.this, "连接成功", Toast.LENGTH_SHORT).show();
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    isConnected = false;
                    btnConnect.setEnabled(true);
                    btnConnect.setText("连接");
                    tvConnectionStatus.setText("连接失败: " + error);
                    setFunctionCardsEnabled(false);
                    
                    Toast.makeText(MainActivity.this, "连接失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void disconnect() {
        isConnected = false;
        btnConnect.setText("连接");
        tvConnectionStatus.setText("未连接");
        setFunctionCardsEnabled(false);
        
        Toast.makeText(this, "已断开连接", Toast.LENGTH_SHORT).show();
    }
    
    private void setFunctionCardsEnabled(boolean enabled) {
        float alpha = enabled ? 1.0f : 0.5f;
        cardRobotControl.setAlpha(alpha);
        cardMapDisplay.setAlpha(alpha);
        cardPointCloud.setAlpha(alpha);
        cardFileManager.setAlpha(alpha);
        
        cardRobotControl.setClickable(enabled);
        cardMapDisplay.setClickable(enabled);
        cardPointCloud.setClickable(enabled);
        cardFileManager.setClickable(enabled);
    }
    
    private void showNotConnectedMessage() {
        Toast.makeText(this, "请先连接到机器人", Toast.LENGTH_SHORT).show();
    }
    
    @Override
    protected void onResume() {
        super.onResume();
        
        // 如果已连接，检查连接状态
        if (isConnected) {
            checkConnectionStatus();
        }
    }
    
    private void checkConnectionStatus() {
        apiClient.getSystemStatus(new ApiCallback<SystemStatus>() {
            @Override
            public void onSuccess(SystemStatus result) {
                // 连接正常，不需要做什么
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    // 连接已断开
                    isConnected = false;
                    btnConnect.setText("连接");
                    tvConnectionStatus.setText("连接已断开");
                    setFunctionCardsEnabled(false);
                    
                    Toast.makeText(MainActivity.this, "连接已断开", Toast.LENGTH_SHORT).show();
                });
            }
        });
    }
    
    public static String getServerIp(android.content.Context context) {
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        return prefs.getString(PREF_SERVER_IP, DEFAULT_SERVER_IP);
    }
}
