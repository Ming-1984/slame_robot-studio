package com.robotstudio.controller;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

/**
 * 设置活动
 */
public class SettingsActivity extends AppCompatActivity {
    
    private static final String PREFS_NAME = "RobotStudioPrefs";
    private static final String PREF_SERVER_IP = "server_ip";
    private static final String PREF_AUTO_CONNECT = "auto_connect";
    private static final String PREF_KEEP_SCREEN_ON = "keep_screen_on";
    private static final String DEFAULT_SERVER_IP = "192.168.4.1";
    
    private EditText etServerIp;
    private Switch switchAutoConnect, switchKeepScreenOn;
    private Button btnSave, btnReset, btnTestConnection;
    private TextView tvAppVersion;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);
        
        initViews();
        loadSettings();
        setupControls();
    }
    
    private void initViews() {
        etServerIp = findViewById(R.id.et_server_ip);
        switchAutoConnect = findViewById(R.id.switch_auto_connect);
        switchKeepScreenOn = findViewById(R.id.switch_keep_screen_on);
        btnSave = findViewById(R.id.btn_save);
        btnReset = findViewById(R.id.btn_reset);
        btnTestConnection = findViewById(R.id.btn_test_connection);
        tvAppVersion = findViewById(R.id.tv_app_version);
    }
    
    private void loadSettings() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        
        String serverIp = prefs.getString(PREF_SERVER_IP, DEFAULT_SERVER_IP);
        boolean autoConnect = prefs.getBoolean(PREF_AUTO_CONNECT, false);
        boolean keepScreenOn = prefs.getBoolean(PREF_KEEP_SCREEN_ON, true);
        
        etServerIp.setText(serverIp);
        switchAutoConnect.setChecked(autoConnect);
        switchKeepScreenOn.setChecked(keepScreenOn);
        
        // 显示应用版本
        try {
            String versionName = getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
            tvAppVersion.setText("版本: " + versionName);
        } catch (Exception e) {
            tvAppVersion.setText("版本: 1.0.0");
        }
    }
    
    private void setupControls() {
        btnSave.setOnClickListener(v -> saveSettings());
        
        btnReset.setOnClickListener(v -> {
            new android.app.AlertDialog.Builder(this)
                .setTitle("重置设置")
                .setMessage("确定要重置所有设置吗？")
                .setPositiveButton("确定", (dialog, which) -> resetSettings())
                .setNegativeButton("取消", null)
                .show();
        });
        
        btnTestConnection.setOnClickListener(v -> testConnection());
        
        // 保持屏幕唤醒设置
        switchKeepScreenOn.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            } else {
                getWindow().clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            }
        });
    }
    
    private void saveSettings() {
        String serverIp = etServerIp.getText().toString().trim();
        
        if (serverIp.isEmpty()) {
            Toast.makeText(this, "请输入服务器IP地址", Toast.LENGTH_SHORT).show();
            return;
        }
        
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        
        editor.putString(PREF_SERVER_IP, serverIp);
        editor.putBoolean(PREF_AUTO_CONNECT, switchAutoConnect.isChecked());
        editor.putBoolean(PREF_KEEP_SCREEN_ON, switchKeepScreenOn.isChecked());
        
        editor.apply();
        
        Toast.makeText(this, "设置已保存", Toast.LENGTH_SHORT).show();
        finish();
    }
    
    private void resetSettings() {
        etServerIp.setText(DEFAULT_SERVER_IP);
        switchAutoConnect.setChecked(false);
        switchKeepScreenOn.setChecked(true);
        
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.clear();
        editor.apply();
        
        Toast.makeText(this, "设置已重置", Toast.LENGTH_SHORT).show();
    }
    
    private void testConnection() {
        String serverIp = etServerIp.getText().toString().trim();
        
        if (serverIp.isEmpty()) {
            Toast.makeText(this, "请输入服务器IP地址", Toast.LENGTH_SHORT).show();
            return;
        }
        
        btnTestConnection.setEnabled(false);
        btnTestConnection.setText("测试中...");
        
        // 创建API客户端测试连接
        com.robotstudio.controller.network.ApiClient apiClient = new com.robotstudio.controller.network.ApiClient();
        apiClient.setServerAddress(serverIp);
        
        apiClient.getSystemStatus(new com.robotstudio.controller.network.ApiCallback<com.robotstudio.controller.model.SystemStatus>() {
            @Override
            public void onSuccess(com.robotstudio.controller.model.SystemStatus result) {
                runOnUiThread(() -> {
                    btnTestConnection.setEnabled(true);
                    btnTestConnection.setText("测试连接");
                    Toast.makeText(SettingsActivity.this, "连接成功！", Toast.LENGTH_SHORT).show();
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    btnTestConnection.setEnabled(true);
                    btnTestConnection.setText("测试连接");
                    Toast.makeText(SettingsActivity.this, "连接失败: " + error, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
} 