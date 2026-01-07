# Robot Studio（Windows）

目标：在一个 Windows 窗口里同时管理（统一入口）：
- 你的上位机网页（Robot Studio Web：`http://192.168.4.1:8080`）
- 测绘建图（`aurora_remote.exe`，启动即自动嵌入到同一窗口）

## 位置
- 解决方案：`windows_control_center/RobotStudioControlCenter.sln`
- 项目：`windows_control_center/RobotStudioControlCenter/RobotStudioControlCenter.csproj`

## 构建
```powershell
cd c:\Users\Ming\Documents\Cursor\slame_car\windows_control_center
dotnet build .\RobotStudioControlCenter.sln -c Release
```

## 运行（开发方式）
```powershell
cd c:\Users\Ming\Documents\Cursor\slame_car\windows_control_center
dotnet run --project .\RobotStudioControlCenter\RobotStudioControlCenter.csproj
```

## 使用说明（核心）

### 1) Robot Studio Web
- 打开程序后，`Robot Studio` Tab 会用 WebView2 直接访问 `http://192.168.4.1:8080/`。

### 2) 测绘建图（嵌入同窗）
- 程序启动后会自动启动并嵌入 `.\aurora_app\aurora_remote.exe`（不再需要手动选择 exe）。

目录结构要求（发布包内必须包含）：
```
RobotStudio.exe
aurora_app/
  aurora_remote.exe
  ...
```

### 3) 连接方式
- 连接与建图等功能仍在**测绘建图内部**完成（本控制台只做统一入口与窗口集成）。

## 注意
- “嵌入到同一个窗口”是通过 Win32 `SetParent` 等方式完成，少数程序可能不支持（表现为黑屏/闪退/无法交互）。如果遇到这种情况，我建议先退一步做“同一个控制中心入口 + 测绘建图外部窗口”，再评估是否需要更深层兼容处理。
- WebView2 需要安装 **Microsoft Edge WebView2 Runtime**（如果你的系统没有，会提示安装）。
- 如果测绘建图启动失败且提示缺少组件，建议先安装 `aurora_app/VC_redist.x64.exe`（发布包也可额外提供 `installers/` 目录用于一键安装）。
