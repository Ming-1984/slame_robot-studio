# slame_robot-studio（Robot Studio）

本仓库用于托管 **Robot Studio**（Windows 控制中心）以及后续的小车端上位机相关代码。

## 目录结构
- Windows 控制中心：`windows_control_center/`
- 一键打包脚本：`scripts/package.ps1`
- 官方测绘建图发行包占位：`aurora_app/`（仓库不提交二进制，见 `aurora_app/README.md`）

## Windows 构建
```powershell
cd windows_control_center
dotnet build .\RobotStudioControlCenter.sln -c Release
```

## Windows 打包（绿色免安装）
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\package.ps1
```

