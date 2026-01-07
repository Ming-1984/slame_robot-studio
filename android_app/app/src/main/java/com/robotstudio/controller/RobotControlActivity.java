package com.robotstudio.controller;

import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.robotstudio.controller.network.ApiClient;
import com.robotstudio.controller.network.ApiCallback;
import com.robotstudio.controller.model.VelocityCommand;
import com.robotstudio.controller.model.SystemCommand;
import com.robotstudio.controller.model.RobotPose;

/**
 * 机器人控制活动
 */
public class RobotControlActivity extends AppCompatActivity {
    
    private ApiClient apiClient;
    private Handler handler;
    
    // UI组件
    private Button btnForward, btnBackward, btnLeft, btnRight, btnStop;
    private Button btnEmergencyStop, btnStartNav, btnStopNav, btnShutdown;
    private SeekBar seekBarSpeed;
    private Switch switchSlowMode;
    private TextView tvSpeed, tvPose, tvStatus;
    
    // 控制参数
    private double maxLinearSpeed = 0.5;  // m/s
    private double maxAngularSpeed = 1.0; // rad/s
    private double currentSpeed = 0.3;
    private boolean isSlowMode = false;
    private boolean isMoving = false;
    
    // 状态更新
    private Runnable statusUpdateRunnable;
    private static final int STATUS_UPDATE_INTERVAL = 1000; // 1秒
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_robot_control);
        
        initViews();
        setupControls();
        
        // 初始化API客户端
        apiClient = new ApiClient();
        String serverIp = MainActivity.getServerIp(this);
        apiClient.setServerAddress(serverIp);
        
        handler = new Handler(Looper.getMainLooper());
        
        startStatusUpdates();
    }
    
    private void initViews() {
        // 方向控制按钮
        btnForward = findViewById(R.id.btn_forward);
        btnBackward = findViewById(R.id.btn_backward);
        btnLeft = findViewById(R.id.btn_left);
        btnRight = findViewById(R.id.btn_right);
        btnStop = findViewById(R.id.btn_stop);
        
        // 系统控制按钮
        btnEmergencyStop = findViewById(R.id.btn_emergency_stop);
        btnStartNav = findViewById(R.id.btn_start_nav);
        btnStopNav = findViewById(R.id.btn_stop_nav);
        btnShutdown = findViewById(R.id.btn_shutdown); // 新增关机按钮
        
        // 速度控制
        seekBarSpeed = findViewById(R.id.seekbar_speed);
        switchSlowMode = findViewById(R.id.switch_slow_mode);
        
        // 状态显示
        tvSpeed = findViewById(R.id.tv_speed);
        tvPose = findViewById(R.id.tv_pose);
        tvStatus = findViewById(R.id.tv_status);
    }
    
    private void setupControls() {
        // 方向控制按钮 - 使用触摸事件实现按住控制
        setupDirectionButton(btnForward, () -> VelocityCommand.forward(getCurrentLinearSpeed()));
        setupDirectionButton(btnBackward, () -> VelocityCommand.backward(getCurrentLinearSpeed()));
        setupDirectionButton(btnLeft, () -> VelocityCommand.turnLeft(getCurrentAngularSpeed()));
        setupDirectionButton(btnRight, () -> VelocityCommand.turnRight(getCurrentAngularSpeed()));
        
        // 停止按钮
        btnStop.setOnClickListener(v -> stopRobot());
        
        // 紧急停止
        btnEmergencyStop.setOnClickListener(v -> emergencyStop());
        
        // 导航控制
        btnStartNav.setOnClickListener(v -> executeSystemCommand(SystemCommand.startPathPlanning()));
        btnStopNav.setOnClickListener(v -> executeSystemCommand(SystemCommand.stopPathPlanning()));
        
        // 关机按钮
        btnShutdown.setOnClickListener(v -> {
            new android.app.AlertDialog.Builder(this)
                .setTitle("关机确认")
                .setMessage("确定要远程关机吗？")
                .setPositiveButton("确定", (dialog, which) -> {
                    executeSystemCommand(SystemCommand.shutdown());
                    Toast.makeText(this, "已发送关机命令", Toast.LENGTH_SHORT).show();
                })
                .setNegativeButton("取消", null)
                .show();
        });
        
        // 速度控制
        seekBarSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    currentSpeed = (progress / 100.0) * maxLinearSpeed;
                    updateSpeedDisplay();
                }
            }
            
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
        });
        
        // 慢速模式
        switchSlowMode.setOnCheckedChangeListener((buttonView, isChecked) -> {
            isSlowMode = isChecked;
            updateSpeedLimits();
            updateSpeedDisplay();
        });
        
        // 初始化速度显示
        seekBarSpeed.setProgress((int) ((currentSpeed / maxLinearSpeed) * 100));
        updateSpeedDisplay();
    }
    
    private void setupDirectionButton(Button button, VelocityCommandProvider commandProvider) {
        button.setOnTouchListener((v, event) -> {
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                    // 按下时开始移动
                    startMoving(commandProvider.getCommand());
                    v.setPressed(true);
                    return true;
                    
                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_CANCEL:
                    // 松开时停止移动
                    stopRobot();
                    v.setPressed(false);
                    return true;
            }
            return false;
        });
    }
    
    private void startMoving(VelocityCommand command) {
        isMoving = true;
        sendVelocityCommand(command);
    }
    
    private void stopRobot() {
        isMoving = false;
        sendVelocityCommand(VelocityCommand.stop());
    }
    
    private void emergencyStop() {
        stopRobot();
        executeSystemCommand(SystemCommand.emergencyStop());
        Toast.makeText(this, "紧急停止已执行", Toast.LENGTH_SHORT).show();
    }
    
    private void sendVelocityCommand(VelocityCommand command) {
        // 应用速度限制
        VelocityCommand clampedCommand = command.clamp(getCurrentLinearSpeed(), getCurrentAngularSpeed());
        
        apiClient.sendVelocityCommand(clampedCommand, new ApiCallback<String>() {
            @Override
            public void onSuccess(String result) {
                // 成功发送，不需要特别处理
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(RobotControlActivity.this, "控制命令发送失败: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        });
    }
    
    private void executeSystemCommand(SystemCommand command) {
        apiClient.executeSystemCommand(command, new ApiCallback<String>() {
            @Override
            public void onSuccess(String result) {
                runOnUiThread(() -> {
                    Toast.makeText(RobotControlActivity.this, "命令执行成功", Toast.LENGTH_SHORT).show();
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(RobotControlActivity.this, "命令执行失败: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        });
    }
    
    private double getCurrentLinearSpeed() {
        return isSlowMode ? currentSpeed * 0.5 : currentSpeed;
    }
    
    private double getCurrentAngularSpeed() {
        return isSlowMode ? maxAngularSpeed * 0.5 : maxAngularSpeed;
    }
    
    private void updateSpeedLimits() {
        if (isSlowMode) {
            maxLinearSpeed = 0.3;
            maxAngularSpeed = 0.6;
        } else {
            maxLinearSpeed = 0.5;
            maxAngularSpeed = 1.0;
        }
    }
    
    private void updateSpeedDisplay() {
        String speedText = String.format("速度: %.1f m/s", getCurrentLinearSpeed());
        if (isSlowMode) {
            speedText += " (慢速模式)";
        }
        tvSpeed.setText(speedText);
    }
    
    private void startStatusUpdates() {
        statusUpdateRunnable = new Runnable() {
            @Override
            public void run() {
                updateRobotPose();
                handler.postDelayed(this, STATUS_UPDATE_INTERVAL);
            }
        };
        handler.post(statusUpdateRunnable);
    }
    
    private void updateRobotPose() {
        apiClient.getRobotPose(new ApiCallback<RobotPose>() {
            @Override
            public void onSuccess(RobotPose pose) {
                runOnUiThread(() -> {
                    String poseText = String.format("位置: (%.2f, %.2f)\n角度: %.1f°", 
                            pose.getX(), pose.getY(), pose.getYawDegrees());
                    tvPose.setText(poseText);
                    tvStatus.setText("连接正常");
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    tvStatus.setText("连接异常: " + error);
                });
            }
        });
    }
    
    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (handler != null && statusUpdateRunnable != null) {
            handler.removeCallbacks(statusUpdateRunnable);
        }
        // 确保停止机器人
        if (isMoving) {
            stopRobot();
        }
    }
    
    @Override
    protected void onPause() {
        super.onPause();
        // 暂停时停止机器人
        if (isMoving) {
            stopRobot();
        }
    }
    
    // 内部接口
    private interface VelocityCommandProvider {
        VelocityCommand getCommand();
    }
}
