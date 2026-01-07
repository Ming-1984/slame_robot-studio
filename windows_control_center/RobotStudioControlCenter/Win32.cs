using System.Diagnostics.CodeAnalysis;
using System.Runtime.InteropServices;
using System.Text;

namespace RobotStudioControlCenter;

internal static class Win32
{
    internal const int SW_HIDE = 0;
    internal const int SW_SHOW = 5;
    internal const int SW_SHOWNOACTIVATE = 4;

    internal const uint EVENT_OBJECT_CREATE = 0x8000;
    internal const uint EVENT_OBJECT_SHOW = 0x8002;
    internal const int OBJID_WINDOW = 0x00000000;

    internal const uint WINEVENT_OUTOFCONTEXT = 0x0000;
    internal const uint WINEVENT_SKIPOWNTHREAD = 0x0001;
    internal const uint WINEVENT_SKIPOWNPROCESS = 0x0002;

    internal const uint STARTF_USESHOWWINDOW = 0x00000001;
    internal const uint STARTF_USEPOSITION = 0x00000004;

    internal const uint CREATE_SUSPENDED = 0x00000004;

    internal const int GWL_STYLE = -16;
    internal const int WS_CHILD = 0x40000000;
    internal const int WS_BORDER = 0x00800000;
    internal const int WS_DLGFRAME = 0x00400000;
    internal const int WS_CAPTION = WS_BORDER | WS_DLGFRAME;
    internal const int WS_THICKFRAME = 0x00040000;
    internal const int WS_MINIMIZEBOX = 0x00020000;
    internal const int WS_MAXIMIZEBOX = 0x00010000;
    internal const int WS_SYSMENU = 0x00080000;

    internal const int SWP_NOSIZE = 0x0001;
    internal const int SWP_NOMOVE = 0x0002;
    internal const int SWP_NOZORDER = 0x0004;
    internal const int SWP_NOACTIVATE = 0x0010;
    internal const int SWP_FRAMECHANGED = 0x0020;
    internal const int SWP_SHOWWINDOW = 0x0040;

    internal const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
    internal const uint MOUSEEVENTF_LEFTUP = 0x0004;

    [DllImport("user32.dll")]
    internal static extern IntPtr SetParent(IntPtr hWndChild, IntPtr hWndNewParent);

    [DllImport("user32.dll")]
    internal static extern IntPtr GetParent(IntPtr hWnd);

    [DllImport("user32.dll", SetLastError = true)]
    internal static extern int SetWindowLong(IntPtr hWnd, int nIndex, int dwNewLong);

    [DllImport("user32.dll", SetLastError = true)]
    internal static extern int GetWindowLong(IntPtr hWnd, int nIndex);

    [DllImport("user32.dll", SetLastError = true)]
    internal static extern bool SetWindowPos(
        IntPtr hWnd,
        IntPtr hWndInsertAfter,
        int X,
        int Y,
        int cx,
        int cy,
        uint uFlags);

    [DllImport("user32.dll", SetLastError = true)]
    internal static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);

    [DllImport("user32.dll")]
    internal static extern bool IsWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    internal static extern bool IsWindowVisible(IntPtr hWnd);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    internal static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);

    [DllImport("user32.dll")]
    internal static extern bool SetForegroundWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    internal static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

    [DllImport("user32.dll")]
    internal static extern IntPtr SetFocus(IntPtr hWnd);

    [DllImport("kernel32.dll")]
    internal static extern uint GetCurrentThreadId();

    [DllImport("user32.dll", SetLastError = true)]
    internal static extern bool AttachThreadInput(uint idAttach, uint idAttachTo, bool fAttach);

    internal const int VK_LBUTTON = 0x01;

    [DllImport("user32.dll")]
    internal static extern short GetAsyncKeyState(int vKey);

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    internal struct STARTUPINFO
    {
        public int cb;
        public string? lpReserved;
        public string? lpDesktop;
        public string? lpTitle;
        public int dwX;
        public int dwY;
        public int dwXSize;
        public int dwYSize;
        public int dwXCountChars;
        public int dwYCountChars;
        public int dwFillAttribute;
        public uint dwFlags;
        public short wShowWindow;
        public short cbReserved2;
        public IntPtr lpReserved2;
        public IntPtr hStdInput;
        public IntPtr hStdOutput;
        public IntPtr hStdError;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct PROCESS_INFORMATION
    {
        public IntPtr hProcess;
        public IntPtr hThread;
        public uint dwProcessId;
        public uint dwThreadId;
    }

    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    internal static extern bool CreateProcess(
        string lpApplicationName,
        StringBuilder? lpCommandLine,
        IntPtr lpProcessAttributes,
        IntPtr lpThreadAttributes,
        bool bInheritHandles,
        uint dwCreationFlags,
        IntPtr lpEnvironment,
        string lpCurrentDirectory,
        ref STARTUPINFO lpStartupInfo,
        out PROCESS_INFORMATION lpProcessInformation);

    [DllImport("kernel32.dll")]
    internal static extern uint ResumeThread(IntPtr hThread);

    [DllImport("kernel32.dll", SetLastError = true)]
    internal static extern bool CloseHandle(IntPtr hObject);

    internal delegate void WinEventDelegate(
        IntPtr hWinEventHook,
        uint eventType,
        IntPtr hwnd,
        int idObject,
        int idChild,
        uint dwEventThread,
        uint dwmsEventTime);

    [DllImport("user32.dll")]
    internal static extern IntPtr SetWinEventHook(
        uint eventMin,
        uint eventMax,
        IntPtr hmodWinEventProc,
        WinEventDelegate lpfnWinEventProc,
        uint idProcess,
        uint idThread,
        uint dwFlags);

    [DllImport("user32.dll")]
    internal static extern bool UnhookWinEvent(IntPtr hWinEventHook);

    [DllImport("user32.dll")]
    internal static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, UIntPtr dwExtraInfo);

    internal delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

    [DllImport("user32.dll")]
    internal static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);

    [DllImport("user32.dll")]
    internal static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);

    [StructLayout(LayoutKind.Sequential)]
    internal struct RECT
    {
        public int Left;
        public int Top;
        public int Right;
        public int Bottom;
    }

    internal static string GetWindowTitle(IntPtr hWnd)
    {
        var sb = new StringBuilder(512);
        _ = GetWindowText(hWnd, sb, sb.Capacity);
        return sb.ToString();
    }

    internal static bool TryGetWindowRect(IntPtr hWnd, [NotNullWhen(true)] out Rectangle? rect)
    {
        rect = null;
        if (hWnd == IntPtr.Zero || !IsWindow(hWnd) || !GetWindowRect(hWnd, out var r))
        {
            return false;
        }

        rect = Rectangle.FromLTRB(r.Left, r.Top, r.Right, r.Bottom);
        return true;
    }

    internal static List<IntPtr> FindTopLevelWindowsByProcessId(int processId)
    {
        var handles = new List<IntPtr>();
        _ = EnumWindows((hWnd, _) =>
        {
            GetWindowThreadProcessId(hWnd, out var pid);
            if (pid == (uint)processId)
            {
                handles.Add(hWnd);
            }

            return true;
        }, IntPtr.Zero);

        return handles;
    }
}
