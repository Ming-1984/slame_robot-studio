#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aurora Point Cloud Processing GUI
A graphical interface for the Aurora point cloud processing pipeline
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import os
import sys
import subprocess
import threading
import queue
import time
import platform
from pathlib import Path

class AuroraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Aurora Point Cloud Processing Tool")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # Get project directory
        self.project_dir = Path(__file__).parent.absolute()
        self.scripts_dir = self.project_dir / "scripts"
        self.data_dir = self.project_dir / "data"
        
        # Output directory (default to project data directory)
        self.output_dir = tk.StringVar(value=str(self.data_dir))
        
        # Queue for thread communication
        self.output_queue = queue.Queue()
        
        # Current process
        self.current_process = None
        
        self.setup_ui()
        self.check_queue()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(7, weight=1)  # Log frame should expand
        
        # Title
        title_label = ttk.Label(main_frame, text="Aurora Point Cloud Processing Tool", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Aurora device IP configuration
        ttk.Label(main_frame, text="Aurora Device IP:").grid(row=1, column=0, sticky=tk.W, pady=5)

        device_frame = ttk.Frame(main_frame)
        device_frame.grid(row=1, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        device_frame.columnconfigure(0, weight=1)

        self.device_ip = tk.StringVar(value="192.168.11.1")
        self.device_entry = ttk.Entry(device_frame, textvariable=self.device_ip, width=50)
        self.device_entry.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 5))

        # Output directory selection
        ttk.Label(main_frame, text="Output Directory:").grid(row=2, column=0, sticky=tk.W, pady=5)

        output_frame = ttk.Frame(main_frame)
        output_frame.grid(row=2, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        output_frame.columnconfigure(0, weight=1)

        self.output_entry = ttk.Entry(output_frame, textvariable=self.output_dir, width=50)
        self.output_entry.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 5))

        ttk.Button(output_frame, text="Browse",
                  command=self.browse_output_dir).grid(row=0, column=1)
        
        # Function buttons frame
        buttons_frame = ttk.LabelFrame(main_frame, text="Processing Functions", padding="10")
        buttons_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        buttons_frame.columnconfigure(0, weight=1)
        buttons_frame.columnconfigure(1, weight=1)
        buttons_frame.columnconfigure(2, weight=1)
        
        # Function buttons
        self.extract_btn = ttk.Button(buttons_frame, text="1. Extract Point Cloud\nfrom Aurora Device",
                                     command=self.extract_point_cloud, width=20)
        self.extract_btn.grid(row=0, column=0, padx=5, pady=5)
        
        self.process_btn = ttk.Button(buttons_frame, text="2. Process Point Cloud\n(Wall Enhancement)",
                                     command=self.process_point_cloud, width=20)
        self.process_btn.grid(row=0, column=1, padx=5, pady=5)

        # Advanced building detection button (row 1)
        self.advanced_btn = ttk.Button(buttons_frame, text="2b. Advanced Detection\n(Doors, Windows, etc.)",
                                      command=self.run_advanced_detection, width=20)
        self.advanced_btn.grid(row=1, column=1, padx=5, pady=5)

        # Advanced detection options frame
        options_frame = ttk.LabelFrame(main_frame, text="Advanced Detection Options", padding="5")
        options_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        # Detection type checkboxes
        self.detect_doors = tk.BooleanVar(value=True)
        self.detect_windows = tk.BooleanVar(value=True)
        self.detect_floors = tk.BooleanVar(value=True)
        self.detect_ceilings = tk.BooleanVar(value=True)

        ttk.Checkbutton(options_frame, text="🚪 Detect Doors", variable=self.detect_doors).grid(row=0, column=0, sticky=tk.W, padx=5)
        ttk.Checkbutton(options_frame, text="🪟 Detect Windows", variable=self.detect_windows).grid(row=0, column=1, sticky=tk.W, padx=5)
        ttk.Checkbutton(options_frame, text="🟢 Detect Floors", variable=self.detect_floors).grid(row=0, column=2, sticky=tk.W, padx=5)
        ttk.Checkbutton(options_frame, text="🔴 Detect Ceilings", variable=self.detect_ceilings).grid(row=0, column=3, sticky=tk.W, padx=5)
        
        # Control buttons frame
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=5, column=0, columnspan=3, pady=10)

        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(main_frame, variable=self.progress_var,
                                           mode='indeterminate')
        self.progress_bar.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

        # Log output area
        log_frame = ttk.LabelFrame(main_frame, text="Process Log", padding="5")
        log_frame.grid(row=7, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=80)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.clear_log_btn = ttk.Button(control_frame, text="Clear Log",
                                       command=self.clear_log)
        self.clear_log_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(control_frame, text="Stop Process",
                                  command=self.stop_process, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.open_output_btn = ttk.Button(control_frame, text="Open Output Folder",
                                         command=self.open_output_folder)
        self.open_output_btn.pack(side=tk.LEFT, padx=5)

        # Batch processing button
        self.batch_btn = ttk.Button(control_frame, text="Run All Steps",
                                   command=self.run_batch_process)
        self.batch_btn.pack(side=tk.LEFT, padx=5)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var,
                              relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=7, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 0))
        
    def browse_output_dir(self):
        """Browse for output directory"""
        directory = filedialog.askdirectory(initialdir=self.output_dir.get())
        if directory:
            self.output_dir.set(directory)
            self.log_message(f"Output directory set to: {directory}")
    
    def log_message(self, message):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()
    
    def clear_log(self):
        """Clear the log text"""
        self.log_text.delete(1.0, tk.END)
    
    def set_buttons_state(self, state):
        """Enable or disable function buttons"""
        self.extract_btn.config(state=state)
        self.process_btn.config(state=state)
        self.batch_btn.config(state=state)
        self.open_output_btn.config(state=state)
        self.stop_btn.config(state=tk.NORMAL if state == tk.DISABLED else tk.DISABLED)
    
    def start_progress(self):
        """Start progress bar animation"""
        self.progress_bar.start(10)
    
    def stop_progress(self):
        """Stop progress bar animation"""
        self.progress_bar.stop()
    
    def check_queue(self):
        """Check for messages from worker threads"""
        try:
            while True:
                message = self.output_queue.get_nowait()
                if message == "PROCESS_COMPLETE":
                    self.on_process_complete()
                elif message == "PROCESS_ERROR":
                    self.on_process_error()
                elif message.startswith("STATUS:"):
                    self.status_var.set(message[7:])
                else:
                    self.log_message(message)
        except queue.Empty:
            pass
        
        # Schedule next check
        self.root.after(100, self.check_queue)
    
    def on_process_complete(self):
        """Handle process completion"""
        if hasattr(self, 'batch_step'):
            # Handle batch processing
            if self.batch_step == 1:
                self.batch_step = 2
                self.log_message("Step 1 completed. Starting step 2...")
                self.process_point_cloud()
                return
            elif self.batch_step == 2:
                self.log_message("=" * 60)
                self.log_message("BATCH PROCESSING COMPLETED SUCCESSFULLY!")
                self.log_message("=" * 60)
                delattr(self, 'batch_step')
                messagebox.showinfo("Success", "All processing steps completed successfully!")

        self.set_buttons_state(tk.NORMAL)
        self.stop_progress()
        self.status_var.set("Process completed successfully")
        self.log_message("Process completed successfully!")

    def on_process_error(self):
        """Handle process error"""
        if hasattr(self, 'batch_step'):
            self.log_message("=" * 60)
            self.log_message("BATCH PROCESSING FAILED!")
            self.log_message("=" * 60)
            delattr(self, 'batch_step')
            messagebox.showerror("Error", "Batch processing failed. Check the log for details.")

        self.set_buttons_state(tk.NORMAL)
        self.stop_progress()
        self.status_var.set("Process failed")
        self.log_message("Process failed!")
    
    def stop_process(self):
        """Stop the current process"""
        if self.current_process:
            try:
                self.current_process.terminate()
                self.log_message("Process terminated by user")
                self.on_process_error()
            except:
                pass
    
    def run_script(self, script_name, description, extra_args=None):
        """Run a script (PowerShell on Windows, Bash on Linux) in a separate thread"""
        def worker():
            try:
                # Determine script path based on OS
                if platform.system() == "Windows":
                    script_path = self.scripts_dir / script_name
                else:
                    # Convert .ps1 to .sh for Linux
                    if script_name.endswith('.ps1'):
                        script_name_linux = script_name.replace('.ps1', '.sh')
                    else:
                        script_name_linux = script_name + '.sh'
                    script_path = self.scripts_dir / script_name_linux

                if not script_path.exists():
                    self.output_queue.put(f"Error: Script not found: {script_path}")
                    self.output_queue.put("PROCESS_ERROR")
                    return

                # On Linux, ensure script is executable
                if platform.system() != "Windows":
                    try:
                        os.chmod(script_path, 0o755)
                    except Exception as e:
                        self.output_queue.put(f"Warning: Could not set script permissions: {e}")

                # Ensure output directory exists
                output_dir = Path(self.output_dir.get())
                if not output_dir.exists():
                    try:
                        output_dir.mkdir(parents=True, exist_ok=True)
                        self.output_queue.put(f"Created output directory: {output_dir}")
                    except Exception as e:
                        self.output_queue.put(f"Error creating output directory: {e}")
                        self.output_queue.put("PROCESS_ERROR")
                        return

                self.output_queue.put(f"Starting {description}...")
                self.output_queue.put(f"Output directory: {output_dir}")
                self.output_queue.put(f"STATUS:Running {description}")

                # Build command based on OS
                if platform.system() == "Windows":
                    cmd = ["powershell.exe", "-ExecutionPolicy", "Bypass", "-File", str(script_path)]
                else:
                    cmd = ["bash", str(script_path)]

                cmd.extend(["-OutputDir", str(output_dir)])

                if extra_args:
                    cmd.extend(extra_args)

                self.current_process = subprocess.Popen(
                    cmd,
                    cwd=str(self.project_dir),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    universal_newlines=True,
                    bufsize=1
                )

                # Read output line by line
                for line in iter(self.current_process.stdout.readline, ''):
                    if line.strip():
                        self.output_queue.put(line.strip())

                self.current_process.wait()

                if self.current_process.returncode == 0:
                    self.output_queue.put("PROCESS_COMPLETE")
                else:
                    self.output_queue.put(f"Process exited with code: {self.current_process.returncode}")
                    self.output_queue.put("PROCESS_ERROR")

            except Exception as e:
                self.output_queue.put(f"Error running script: {str(e)}")
                self.output_queue.put("PROCESS_ERROR")

        # Start worker thread
        self.set_buttons_state(tk.DISABLED)
        self.start_progress()
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
    
    def extract_point_cloud(self):
        """Extract point cloud from Aurora device"""
        self.log_message("=" * 50)
        self.log_message("Starting Point Cloud Extraction from Aurora Device")
        self.log_message(f"Aurora Device IP: {self.device_ip.get()}")
        self.log_message("Connecting to Aurora device and extracting point cloud data...")
        self.log_message("This may take several minutes depending on the map size.")

        self.run_script("extract_point_cloud.ps1", "Point Cloud Extraction",
                       extra_args=["-DeviceIP", self.device_ip.get()])
    
    def process_point_cloud(self):
        """Process point cloud with wall enhancement"""
        self.log_message("=" * 50)
        self.log_message("Starting Point Cloud Processing (Wall Enhancement)")
        self.log_message("This will automatically run advanced building element detection")
        self.run_script("process_point_cloud_walls.ps1", "Point Cloud Processing", ["-AutoAdvanced"])

    def run_advanced_detection(self):
        """Run advanced building element detection"""
        self.log_message("=" * 50)
        self.log_message("🏗️ Starting Advanced Building Element Detection")
        self.log_message("This will detect and color-code:")
        self.log_message("🧱 Walls (Gray), 🚪 Doors (Brown), 🪟 Windows (Blue)")
        self.log_message("🟢 Floors (Green), 🔴 Ceilings (Red)")
        self.run_script("advanced_building_detection.ps1", "Advanced Building Detection")

    def create_tooltip(self, widget, text):
        """Create a simple tooltip for a widget"""
        def on_enter(event):
            tooltip = tk.Toplevel()
            tooltip.wm_overrideredirect(True)
            tooltip.wm_geometry(f"+{event.x_root+10}+{event.y_root+10}")
            label = tk.Label(tooltip, text=text, background="lightyellow",
                           relief="solid", borderwidth=1, font=("Arial", 9))
            label.pack()
            widget.tooltip = tooltip

        def on_leave(event):
            _ = event  # Suppress unused parameter warning
            if hasattr(widget, 'tooltip'):
                widget.tooltip.destroy()
                del widget.tooltip

        widget.bind("<Enter>", on_enter)
        widget.bind("<Leave>", on_leave)

    def open_output_folder(self):
        """Open the output folder in file explorer"""
        output_path = Path(self.output_dir.get())
        if output_path.exists():
            try:
                # Cross-platform file manager opening
                if platform.system() == "Windows":
                    os.startfile(str(output_path))
                elif platform.system() == "Darwin":  # macOS
                    subprocess.run(["open", str(output_path)])
                else:  # Linux and other Unix-like systems
                    subprocess.run(["xdg-open", str(output_path)])
                self.log_message(f"Opened output folder: {output_path}")
            except Exception as e:
                self.log_message(f"Error opening folder: {e}")
                messagebox.showerror("Error", f"Could not open folder: {e}")
        else:
            messagebox.showwarning("Warning", f"Output folder does not exist: {output_path}")

    def run_batch_process(self):
        """Run all processing steps in sequence"""
        if messagebox.askyesno("Batch Process",
                              "This will run all processing steps in sequence:\n"
                              "1. Extract Point Cloud\n"
                              "2. Process Point Cloud (Wall Enhancement)\n"
                              "This may take several minutes. Continue?"):
            self.batch_step = 1
            self.log_message("=" * 60)
            self.log_message("STARTING BATCH PROCESSING")
            self.log_message("=" * 60)
            self.extract_point_cloud()



def main():
    """Main function"""
    root = tk.Tk()
    app = AuroraGUI(root)
    
    # Handle window closing
    def on_closing():
        if app.current_process:
            if messagebox.askokcancel("Quit", "A process is running. Do you want to terminate it and quit?"):
                app.stop_process()
                root.destroy()
        else:
            root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
