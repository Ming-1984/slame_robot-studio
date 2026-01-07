package com.robotstudio.controller;

import android.app.DownloadManager;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.robotstudio.controller.network.ApiClient;
import com.robotstudio.controller.network.ApiCallback;
import com.robotstudio.controller.model.FileInfo;

import java.util.ArrayList;
import java.util.List;

/**
 * 文件管理活动
 */
public class FileManagerActivity extends AppCompatActivity {
    
    private ApiClient apiClient;
    
    // UI组件
    private Button btnMaps, btnClouds, btnRefresh;
    private RecyclerView recyclerViewFiles;
    private ProgressBar progressBar;
    private TextView tvStatus, tvFileCount;
    
    // 数据
    private FileAdapter fileAdapter;
    private List<FileInfo> fileList;
    private String currentFileType = "maps";
    
    // 下载管理
    private DownloadManager downloadManager;
    private BroadcastReceiver downloadReceiver;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_file_manager);
        
        initViews();
        setupRecyclerView();
        setupControls();
        
        // 初始化API客户端
        apiClient = new ApiClient();
        String serverIp = MainActivity.getServerIp(this);
        apiClient.setServerAddress(serverIp);
        
        // 初始化下载管理器
        downloadManager = (DownloadManager) getSystemService(Context.DOWNLOAD_SERVICE);
        setupDownloadReceiver();
        
        // 默认加载地图文件
        loadFileList();
    }
    
    private void initViews() {
        btnMaps = findViewById(R.id.btn_maps);
        btnClouds = findViewById(R.id.btn_clouds);
        btnRefresh = findViewById(R.id.btn_refresh);
        recyclerViewFiles = findViewById(R.id.recycler_view_files);
        progressBar = findViewById(R.id.progress_bar);
        tvStatus = findViewById(R.id.tv_status);
        tvFileCount = findViewById(R.id.tv_file_count);
    }
    
    private void setupRecyclerView() {
        fileList = new ArrayList<>();
        fileAdapter = new FileAdapter(fileList);
        recyclerViewFiles.setLayoutManager(new LinearLayoutManager(this));
        recyclerViewFiles.setAdapter(fileAdapter);
    }
    
    private void setupControls() {
        btnMaps.setOnClickListener(v -> {
            currentFileType = "maps";
            updateButtonStates();
            loadFileList();
        });
        
        btnClouds.setOnClickListener(v -> {
            currentFileType = "clouds";
            updateButtonStates();
            loadFileList();
        });
        
        btnRefresh.setOnClickListener(v -> loadFileList());
    }
    
    private void updateButtonStates() {
        btnMaps.setSelected("maps".equals(currentFileType));
        btnClouds.setSelected("clouds".equals(currentFileType));
    }
    
    private void loadFileList() {
        progressBar.setVisibility(View.VISIBLE);
        tvStatus.setText("正在加载文件列表...");
        
        apiClient.getFileList(currentFileType, new ApiCallback<List<FileInfo>>() {
            @Override
            public void onSuccess(List<FileInfo> files) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    fileList.clear();
                    fileList.addAll(files);
                    fileAdapter.notifyDataSetChanged();
                    
                    String statusText = "maps".equals(currentFileType) ? "地图文件" : "点云文件";
                    tvStatus.setText(statusText + "加载完成");
                    tvFileCount.setText("共 " + files.size() + " 个文件");
                    
                    if (files.isEmpty()) {
                        Toast.makeText(FileManagerActivity.this, "没有找到" + statusText, Toast.LENGTH_SHORT).show();
                    }
                });
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    tvStatus.setText("加载失败: " + error);
                    tvFileCount.setText("0 个文件");
                    
                    // 更详细的错误提示
                    String errorMessage = "加载文件列表失败";
                    if (error.contains("网络连接失败")) {
                        errorMessage += "：请检查网络连接";
                    } else if (error.contains("服务器错误")) {
                        errorMessage += "：服务器暂时不可用";
                    } else {
                        errorMessage += "：" + error;
                    }
                    
                    Toast.makeText(FileManagerActivity.this, errorMessage, Toast.LENGTH_LONG).show();
                });
            }
        });
    }
    
    private void downloadFile(FileInfo fileInfo) {
        try {
            // 检查文件大小
            if (fileInfo.getSize() > 100 * 1024 * 1024) { // 100MB
                new android.app.AlertDialog.Builder(this)
                    .setTitle("大文件下载")
                    .setMessage("文件较大(" + fileInfo.getFormattedSize() + ")，下载可能需要较长时间，是否继续？")
                    .setPositiveButton("继续下载", (dialog, which) -> startDownload(fileInfo))
                    .setNegativeButton("取消", null)
                    .show();
            } else {
                startDownload(fileInfo);
            }
            
        } catch (Exception e) {
            Toast.makeText(this, "下载失败: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }
    
    private void startDownload(FileInfo fileInfo) {
        try {
            String downloadUrl = apiClient.getDownloadUrl(currentFileType, fileInfo.getName());
            String fileName = fileInfo.getName() + ".zip";
            
            // 检查存储权限
            if (!checkStoragePermission()) {
                Toast.makeText(this, "需要存储权限才能下载文件", Toast.LENGTH_LONG).show();
                return;
            }
            
            DownloadManager.Request request = new DownloadManager.Request(Uri.parse(downloadUrl));
            request.setTitle("下载 " + fileInfo.getName());
            request.setDescription("正在下载" + ("maps".equals(currentFileType) ? "地图" : "点云") + "文件");
            request.setNotificationVisibility(DownloadManager.Request.VISIBILITY_VISIBLE_NOTIFY_COMPLETED);
            request.setDestinationInExternalPublicDir(Environment.DIRECTORY_DOWNLOADS, fileName);
            request.setAllowedNetworkTypes(DownloadManager.Request.NETWORK_WIFI | DownloadManager.Request.NETWORK_MOBILE);
            
            long downloadId = downloadManager.enqueue(request);
            
            Toast.makeText(this, "开始下载: " + fileName, Toast.LENGTH_SHORT).show();
            
        } catch (Exception e) {
            Toast.makeText(this, "下载失败: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }
    
    private boolean checkStoragePermission() {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.M) {
            if (checkSelfPermission(android.Manifest.permission.WRITE_EXTERNAL_STORAGE) 
                != android.content.pm.PackageManager.PERMISSION_GRANTED) {
                requestPermissions(new String[]{android.Manifest.permission.WRITE_EXTERNAL_STORAGE}, 1001);
                return false;
            }
        }
        return true;
    }
    
    private void setupDownloadReceiver() {
        downloadReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                String action = intent.getAction();
                if (DownloadManager.ACTION_DOWNLOAD_COMPLETE.equals(action)) {
                    Toast.makeText(context, "文件下载完成", Toast.LENGTH_SHORT).show();
                }
            }
        };
        
        registerReceiver(downloadReceiver, new IntentFilter(DownloadManager.ACTION_DOWNLOAD_COMPLETE));
    }
    
    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (downloadReceiver != null) {
            unregisterReceiver(downloadReceiver);
        }
    }
    
    // 文件适配器
    private class FileAdapter extends RecyclerView.Adapter<FileAdapter.FileViewHolder> {
        private List<FileInfo> files;
        
        public FileAdapter(List<FileInfo> files) {
            this.files = files;
        }
        
        @Override
        public FileViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
            View view = LayoutInflater.from(parent.getContext())
                    .inflate(R.layout.item_file, parent, false);
            return new FileViewHolder(view);
        }
        
        @Override
        public void onBindViewHolder(FileViewHolder holder, int position) {
            FileInfo fileInfo = files.get(position);
            holder.bind(fileInfo);
        }
        
        @Override
        public int getItemCount() {
            return files.size();
        }
        
        class FileViewHolder extends RecyclerView.ViewHolder {
            private TextView tvFileName, tvFileSize, tvFileDate;
            private Button btnDownload;
            
            public FileViewHolder(View itemView) {
                super(itemView);
                tvFileName = itemView.findViewById(R.id.tv_file_name);
                tvFileSize = itemView.findViewById(R.id.tv_file_size);
                tvFileDate = itemView.findViewById(R.id.tv_file_date);
                btnDownload = itemView.findViewById(R.id.btn_download);
            }
            
            public void bind(FileInfo fileInfo) {
                tvFileName.setText(fileInfo.getName());
                
                if ("clouds".equals(currentFileType) && fileInfo.getFileCount() > 0) {
                    tvFileSize.setText(fileInfo.getFormattedSize() + " (" + fileInfo.getFileCount() + " 个文件)");
                } else {
                    tvFileSize.setText(fileInfo.getFormattedSize());
                }
                
                tvFileDate.setText(fileInfo.getModified());
                
                btnDownload.setOnClickListener(v -> downloadFile(fileInfo));
            }
        }
    }
}
