"""
Dyno S Controller - Giao diện PC
- Kết nối STM32 qua USB CDC (cổng COM ảo)
- Nhập khối lượng xe, bán kính bánh, chọn chế độ (mô-men thủ công / mô phỏng tải đường)
- Hiển thị và vẽ đồ thị thời gian thực
- Ghi dữ liệu đo và lưu ra file Excel (.xlsx) trong thư mục người dùng chọn

Telemetry từ firmware (8 cột CSV, 50 Hz):
    dong_mA, T_dong_Nm, T_loadcell_Nm, duty_pct, rpm, P_W, v_kmh, Fw_N

Lệnh gửi xuống firmware:
    M<kg>  R<m>  S1/S0  T<Nm>
"""

import os
import queue
import threading
import time
from collections import deque
from datetime import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
import serial.tools.list_ports
from openpyxl import Workbook
from openpyxl.chart import LineChart, Reference
from openpyxl.styles import Font, PatternFill, Alignment
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

COLUMNS = [
    ("current_mA",   "Dòng điện (mA)"),
    ("torque_i_nm",  "Mô-men theo dòng (N.m)"),
    ("torque_lc_nm", "Mô-men loadcell (N.m)"),
    ("duty_pct",     "Duty PWM (%)"),
    ("rpm",          "Tốc độ (RPM)"),
    ("power_w",      "Công suất (W)"),
    ("v_kmh",        "Vận tốc (km/h)"),
    ("fw_n",         "Lực cản Fw (N)"),
]
PLOT_SECONDS = 30            # Cửa sổ thời gian mặc định
PLOT_WINDOWS = (10, 30, 60, 120, 300)
TELEMETRY_HZ = 50
PLOT_INTERVAL_MS = 100       # Tốc độ làm mới đồ thị (10 khung/giây)
PLOT_MAX_POINTS = 1500       # Giảm mẫu khi cửa sổ dài để vẽ nhanh
DEFAULT_PLOTS = ("v_kmh", "torque_lc_nm", "power_w")
PLOT_COLORS = ("tab:blue", "tab:orange", "tab:purple", "tab:gray",
               "tab:cyan", "tab:red", "tab:blue", "tab:green")


class SerialWorker(threading.Thread):
    """Đọc dữ liệu nền, đẩy từng dòng đã phân tích vào queue."""

    def __init__(self, port, out_queue):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.out_queue = out_queue
        self.running = True

    def run(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.out_queue.put(("error", str(e)))
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="ignore").strip()
                if not text:
                    continue
                parts = text.split(",")
                try:
                    values = [float(p) for p in parts]
                except ValueError:
                    self.out_queue.put(("text", text))  # Thông báo dạng chữ (vd: tool calib)
                    continue
                if len(values) >= 6:
                    values += [0.0] * (len(COLUMNS) - len(values))
                    self.out_queue.put(("data", time.time(), values[:len(COLUMNS)]))

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


DEMO_PORT = "DEMO - Dữ liệu giả lập (không cần phần cứng)"


class DemoWorker(threading.Thread):
    """Sinh dữ liệu giả lập 50 Hz: xe tăng tốc rồi giảm tốc, để thử GUI khi chưa có phần cứng."""

    def __init__(self, out_queue):
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self.running = True
        self.mass, self.radius = 150.0, 0.28

    def run(self):
        import math
        import random
        t0 = time.time()
        v_prev = 0.0
        while self.running:
            t = time.time() - t0
            v_kmh = 30.0 * (1 - math.cos(2 * math.pi * t / 20.0))  # chu kỳ 20 s, 0-60 km/h
            a = (v_kmh - v_prev) / 3.6 * TELEMETRY_HZ
            v_prev = v_kmh
            m = self.mass
            fw = m * a + 0.088 * m + (0.000015 * m + 0.02) * v_kmh ** 2
            m_dyno = (0.0915 + 0.16944) / 0.111 ** 2  # quán tính phanh bột + con lăn quy về mặt lăn
            torque = max(0.0, fw - m_dyno * a) * 0.111
            omega = v_kmh / 3.6 / 0.111  # v (m/s) = omega_encoder (rad/s) * 0.111
            wheel_omega = v_kmh / 3.6 / self.radius  # tốc độ quay bánh xe = v / r
            noise = random.uniform(-0.3, 0.3)
            values = [torque / 74.07 * 1000, torque, torque + noise,
                      min(100.0, torque / 2), wheel_omega * 30 / math.pi, torque * omega, v_kmh, fw]
            self.out_queue.put(("data", time.time(), values))
            time.sleep(1.0 / TELEMETRY_HZ)

    def send(self, cmd):
        try:
            if cmd[0] == "M":
                self.mass = float(cmd[1:])
            elif cmd[0] == "R":
                self.radius = float(cmd[1:])
        except ValueError:
            pass

    def stop(self):
        self.running = False


class DynoApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dyno S Controller")
        self.geometry("1280x760")
        self.minsize(1100, 650)

        self.worker = None
        self.rx_queue = queue.Queue()
        self.recording = False
        self.record_rows = []
        self.record_t0 = 0.0
        self.record_params = {}

        n = max(PLOT_WINDOWS) * TELEMETRY_HZ
        self.t_hist = deque(maxlen=n)
        self.hist = {k: deque(maxlen=n) for k, _ in COLUMNS}
        self.t_start = time.time()

        self._build_ui()
        self.refresh_ports()
        self.after(50, self.poll_queue)
        self.after(PLOT_INTERVAL_MS, self.update_plot)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        style = ttk.Style(self)
        style.configure("Big.TLabel", font=("Segoe UI", 16, "bold"))
        style.configure("Stop.TButton", font=("Segoe UI", 12, "bold"), foreground="red")

        left = ttk.Frame(self, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(self, padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- Kết nối
        f = ttk.LabelFrame(left, text="Kết nối", padding=6)
        f.pack(fill=tk.X, pady=4)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(f, textvariable=self.port_var, width=48, state="readonly",
                                    postcommand=self._widen_port_list)
        self.port_cb.grid(row=0, column=0, columnspan=2, sticky="ew")
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        ttk.Button(f, text="Làm mới", command=self.refresh_ports).grid(row=1, column=0, sticky="ew", pady=2)
        self.connect_btn = ttk.Button(f, text="Kết nối", command=self.toggle_connect)
        self.connect_btn.grid(row=1, column=1, sticky="ew", pady=2)
        self.status_var = tk.StringVar(value="Chưa kết nối")
        ttk.Label(f, textvariable=self.status_var, foreground="gray").grid(row=2, column=0, columnspan=2, sticky="w")

        # --- Thông số xe
        f = ttk.LabelFrame(left, text="Thông số xe", padding=6)
        f.pack(fill=tk.X, pady=4)
        self.vehicle_name = tk.StringVar(value="")
        self.mass_var = tk.StringVar(value="150")
        self.radius_var = tk.StringVar(value="0.28")
        self._entry_row(f, 0, "Tên xe / Biển số:", self.vehicle_name, width=16)
        self._entry_row(f, 1, "Khối lượng xe + người (kg):", self.mass_var)
        self._entry_row(f, 2, "Bán kính bánh xe (m):", self.radius_var)
        ttk.Button(f, text="Gửi thông số xuống bộ điều khiển", command=self.send_params)\
            .grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))

        # --- Chế độ
        f = ttk.LabelFrame(left, text="Chế độ điều khiển phanh", padding=6)
        f.pack(fill=tk.X, pady=4)
        self.mode_var = tk.StringVar(value="sim")
        ttk.Radiobutton(f, text="Mô phỏng tải đường (Fw)", variable=self.mode_var, value="sim")\
            .grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Radiobutton(f, text="Mô-men thủ công", variable=self.mode_var, value="manual")\
            .grid(row=1, column=0, columnspan=2, sticky="w")
        self.torque_var = tk.StringVar(value="0")
        self._entry_row(f, 2, "Mô-men đặt (N.m, 0-200):", self.torque_var)
        ttk.Button(f, text="▶ Áp dụng", command=self.apply_mode).grid(row=3, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(f, text="■ DỪNG PHANH", style="Stop.TButton", command=self.stop_brake)\
            .grid(row=3, column=1, sticky="ew", pady=(6, 0))

        # --- Ghi dữ liệu
        f = ttk.LabelFrame(left, text="Ghi dữ liệu đo (Excel)", padding=6)
        f.pack(fill=tk.X, pady=4)
        self.folder_var = tk.StringVar(value=os.path.join(os.path.expanduser("~"), "Documents", "Dyno_Data"))
        self.test_name = tk.StringVar(value="Lan_do")
        ttk.Label(f, text="Thư mục lưu:").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.folder_var, width=30).grid(row=1, column=0, sticky="ew")
        ttk.Button(f, text="Chọn...", command=self.choose_folder).grid(row=1, column=1, padx=(4, 0))
        ttk.Label(f, text="Tên lần đo:").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(f, textvariable=self.test_name, width=30).grid(row=3, column=0, columnspan=2, sticky="ew")
        self.rec_btn = ttk.Button(f, text="● Bắt đầu ghi", command=self.toggle_record)
        self.rec_btn.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        self.rec_var = tk.StringVar(value="Không ghi")
        ttk.Label(f, textvariable=self.rec_var).grid(row=5, column=0, columnspan=2, sticky="w")

        # --- Nhật ký
        f = ttk.LabelFrame(left, text="Nhật ký", padding=6)
        f.pack(fill=tk.BOTH, expand=True, pady=4)
        self.log = tk.Text(f, height=6, width=38, state="disabled", font=("Consolas", 9))
        self.log.pack(fill=tk.BOTH, expand=True)

        # --- Giá trị tức thời
        vals = ttk.Frame(right)
        vals.pack(fill=tk.X)
        self.val_vars = {}
        for i, (key, label) in enumerate(COLUMNS):
            box = ttk.LabelFrame(vals, text=label, padding=4)
            box.grid(row=i // 4, column=i % 4, sticky="ew", padx=3, pady=3)
            vals.columnconfigure(i % 4, weight=1)
            v = tk.StringVar(value="--")
            ttk.Label(box, textvariable=v, style="Big.TLabel").pack()
            self.val_vars[key] = v

        # --- Điều khiển đồ thị: chọn tín hiệu, cửa sổ thời gian, tạm dừng
        ctl = ttk.LabelFrame(right, text="Đồ thị thời gian thực", padding=4)
        ctl.pack(fill=tk.X, pady=(6, 0))
        self.plot_sel = {}
        for i, (key, label) in enumerate(COLUMNS):
            var = tk.BooleanVar(value=key in DEFAULT_PLOTS)
            ttk.Checkbutton(ctl, text=label, variable=var, command=self.rebuild_plots)\
                .grid(row=i // 4, column=i % 4, sticky="w", padx=4)
            self.plot_sel[key] = var
        bar = ttk.Frame(ctl)
        bar.grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))
        ttk.Label(bar, text="Cửa sổ (s):").pack(side=tk.LEFT)
        self.window_var = tk.StringVar(value=str(PLOT_SECONDS))
        ttk.Combobox(bar, textvariable=self.window_var, width=5, state="readonly",
                     values=[str(w) for w in PLOT_WINDOWS]).pack(side=tk.LEFT, padx=(2, 10))
        self.paused = False
        self.pause_btn = ttk.Button(bar, text="⏸ Tạm dừng", command=self.toggle_pause)
        self.pause_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text="Xóa đồ thị", command=self.clear_plot).pack(side=tk.LEFT, padx=2)
        self.rate_var = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.rate_var, foreground="gray").pack(side=tk.LEFT, padx=10)
        self._rate_count = 0
        self._rate_t = time.time()

        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self.plot_lines = {}
        self.rebuild_plots()

    def rebuild_plots(self):
        """Mỗi tín hiệu được chọn có một đồ thị riêng, xếp chồng, chung trục thời gian."""
        self.fig.clear()
        self.plot_lines = {}
        keys = [k for k, _ in COLUMNS if self.plot_sel[k].get()]
        labels = dict(COLUMNS)
        first = None
        for i, key in enumerate(keys):
            ax = self.fig.add_subplot(len(keys), 1, i + 1, sharex=first)
            first = first or ax
            color = PLOT_COLORS[[k for k, _ in COLUMNS].index(key)]
            line, = ax.plot([], [], color=color, linewidth=1.2)
            ax.set_ylabel(labels[key], fontsize=8)
            ax.tick_params(labelsize=8)
            ax.grid(True, alpha=0.3)
            if i < len(keys) - 1:
                ax.tick_params(labelbottom=False)
            else:
                ax.set_xlabel("Thời gian (s)", fontsize=8)
            self.plot_lines[key] = (ax, line)
        if keys:
            self.fig.tight_layout()
        self.canvas.draw_idle()

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.configure(text="▶ Tiếp tục" if self.paused else "⏸ Tạm dừng")

    def clear_plot(self):
        self.t_hist.clear()
        for d in self.hist.values():
            d.clear()
        self.t_start = time.time()
        self.canvas.draw_idle()

    def _entry_row(self, parent, row, label, var, width=10):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=1)
        ttk.Entry(parent, textvariable=var, width=width).grid(row=row, column=1, sticky="e", pady=1)

    def write_log(self, msg):
        self.log.configure(state="normal")
        self.log.insert(tk.END, f"[{datetime.now():%H:%M:%S}] {msg}\n")
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    # ------------------------------------------------------------ Kết nối
    def _widen_port_list(self):
        """Mở rộng danh sách xổ xuống theo tên cổng dài nhất."""
        names = self.port_cb["values"] or ("",)
        width = max(len(n) for n in names) + 2
        popdown = self.port_cb.tk.eval(f"ttk::combobox::PopdownWindow {self.port_cb}")
        self.port_cb.tk.call(f"{popdown}.f.l", "configure", "-width", width)

    def refresh_ports(self):
        ports = [f"{p.device} - {p.description}" for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports + [DEMO_PORT]
        if ports and not self.port_var.get():
            # Ưu tiên cổng STM32 CDC ("STM..." hoặc "USB Serial Device"), bỏ qua Bluetooth
            stm = [p for p in ports if "STM" in p.upper() or "USB" in p.upper()]
            other = [p for p in ports if "BLUETOOTH" not in p.upper()]
            self.port_var.set(stm[0] if stm else (other[0] if other else DEMO_PORT))

    def toggle_connect(self):
        if self.worker:
            self.disconnect()
            return
        sel = self.port_var.get()
        if not sel:
            messagebox.showwarning("Kết nối", "Chưa chọn cổng COM.")
            return
        port = sel.split(" - ")[0]
        if sel == DEMO_PORT:
            self.worker = DemoWorker(self.rx_queue)
        else:
            try:
                self.worker = SerialWorker(port, self.rx_queue)
            except serial.SerialException as e:
                messagebox.showerror("Kết nối", f"Không mở được {port}:\n{e}")
                return
        self.worker.start()
        self.connect_btn.configure(text="Ngắt kết nối")
        self.status_var.set(f"Đã kết nối {port}")
        self.write_log(f"Kết nối {port}")
        self.send_params()

    def disconnect(self):
        if self.recording:
            self.toggle_record()
        if self.worker:
            try:
                self.worker.send("S0")
            except Exception:
                pass
            self.worker.stop()
            self.worker = None
        self.connect_btn.configure(text="Kết nối")
        self.status_var.set("Chưa kết nối")
        self.write_log("Ngắt kết nối")

    def send(self, cmd):
        if not self.worker:
            messagebox.showwarning("Lệnh", "Chưa kết nối bộ điều khiển.")
            return False
        try:
            self.worker.send(cmd)
        except serial.SerialException as e:
            self.write_log(f"Lỗi gửi: {e}")
            return False
        self.write_log(f"Gửi: {cmd}")
        return True

    # ------------------------------------------------------------- Lệnh
    def _read_float(self, var, name, lo, hi):
        try:
            v = float(var.get().replace(",", "."))
        except ValueError:
            messagebox.showerror("Thông số", f"{name} không hợp lệ.")
            return None
        if not lo <= v <= hi:
            messagebox.showerror("Thông số", f"{name} phải trong khoảng {lo} - {hi}.")
            return None
        return v

    def send_params(self):
        m = self._read_float(self.mass_var, "Khối lượng", 30, 1000)
        r = self._read_float(self.radius_var, "Bán kính bánh", 0.10, 0.60)
        if m is None or r is None:
            return False
        return self.send(f"M{m:.2f}") and self.send(f"R{r:.4f}")

    def apply_mode(self):
        if self.mode_var.get() == "sim":
            if self.send_params():
                self.send("S1")
        else:
            t = self._read_float(self.torque_var, "Mô-men", 0, 200)
            if t is not None:
                self.send(f"T{t:.2f}")

    def stop_brake(self):
        if self.worker:
            self.send("S0")
            self.send("T0")

    # ---------------------------------------------------------- Ghi file
    def choose_folder(self):
        d = filedialog.askdirectory(initialdir=self.folder_var.get(), title="Chọn thư mục lưu file đo")
        if d:
            self.folder_var.set(d)

    def toggle_record(self):
        if not self.recording:
            if not self.worker:
                messagebox.showwarning("Ghi dữ liệu", "Chưa kết nối bộ điều khiển.")
                return
            self.record_rows = []
            self.record_t0 = time.time()
            self.record_params = {
                "Tên xe / Biển số": self.vehicle_name.get(),
                "Tên lần đo": self.test_name.get(),
                "Khối lượng (kg)": self.mass_var.get(),
                "Bán kính bánh (m)": self.radius_var.get(),
                "Chế độ": "Mô phỏng tải đường" if self.mode_var.get() == "sim" else "Mô-men thủ công",
                "Mô-men đặt (N.m)": self.torque_var.get() if self.mode_var.get() == "manual" else "-",
                "Bắt đầu": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
            self.recording = True
            self.rec_btn.configure(text="■ Dừng ghi và lưu Excel")
            self.write_log("Bắt đầu ghi dữ liệu")
        else:
            self.recording = False
            self.rec_btn.configure(text="● Bắt đầu ghi")
            self.rec_var.set("Không ghi")
            self.save_excel()

    def save_excel(self):
        if not self.record_rows:
            self.write_log("Không có dữ liệu để lưu")
            return
        folder = self.folder_var.get()
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as e:
            messagebox.showerror("Lưu file", f"Không tạo được thư mục:\n{e}")
            return
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in self.test_name.get()) or "Lan_do"
        path = os.path.join(folder, f"{safe}_{datetime.now():%Y%m%d_%H%M%S}.xlsx")

        wb = Workbook()
        hdr_font = Font(bold=True, color="FFFFFF")
        hdr_fill = PatternFill("solid", fgColor="305496")

        ws = wb.active
        ws.title = "Thong so"
        self.record_params["Kết thúc"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.record_params["Số mẫu"] = len(self.record_rows)
        self.record_params["Thời gian đo (s)"] = round(self.record_rows[-1][0], 2)
        ws.append(["Thông số", "Giá trị"])
        for k, v in self.record_params.items():
            ws.append([k, v])
        for c in ws[1]:
            c.font, c.fill = hdr_font, hdr_fill
        ws.column_dimensions["A"].width = 24
        ws.column_dimensions["B"].width = 28

        wd = wb.create_sheet("Du lieu")
        wd.append(["Thời gian (s)"] + [lbl for _, lbl in COLUMNS])
        for c in wd[1]:
            c.font, c.fill = hdr_font, hdr_fill
            c.alignment = Alignment(wrap_text=True, horizontal="center")
        for row in self.record_rows:
            wd.append(row)
        for col in "ABCDEFGHI":
            wd.column_dimensions[col].width = 15
        wd.freeze_panes = "A2"

        # Biểu đồ trong Excel: vận tốc, mô-men loadcell, công suất theo thời gian
        n = len(self.record_rows) + 1
        x = Reference(wd, min_col=1, min_row=2, max_row=n)
        for title, col, anchor in (("Vận tốc (km/h)", 8, "K2"),
                                   ("Mô-men loadcell (N.m)", 4, "K20"),
                                   ("Công suất (W)", 7, "K38")):
            ch = LineChart()
            ch.title, ch.x_axis.title, ch.height, ch.width = title, "Thời gian (s)", 8, 16
            ch.add_data(Reference(wd, min_col=col, min_row=1, max_row=n), titles_from_data=True)
            ch.set_categories(x)
            ch.legend = None
            wd.add_chart(ch, anchor)

        try:
            wb.save(path)
        except PermissionError:
            messagebox.showerror("Lưu file", f"Không ghi được (file đang mở?):\n{path}")
            return
        self.write_log(f"Đã lưu {len(self.record_rows)} mẫu: {path}")
        messagebox.showinfo("Lưu file", f"Đã lưu file đo:\n{path}")

    # ----------------------------------------------------------- Dữ liệu
    def poll_queue(self):
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if item[0] == "data":
                    _, ts, values = item
                    self._rate_count += 1
                    self.t_hist.append(ts - self.t_start)
                    for (key, _), v in zip(COLUMNS, values):
                        self.hist[key].append(v)
                    if self.recording:
                        self.record_rows.append([round(ts - self.record_t0, 3)] + values)
                elif item[0] == "text":
                    self.write_log(item[1])
                elif item[0] == "error":
                    self.write_log(f"Lỗi cổng COM: {item[1]}")
                    self.disconnect()
        except queue.Empty:
            pass

        if self.t_hist:
            fmt = {"current_mA": "{:.0f}", "duty_pct": "{:.1f}", "rpm": "{:.0f}", "v_kmh": "{:.1f}"}
            for key, _ in COLUMNS:
                self.val_vars[key].set(fmt.get(key, "{:.2f}").format(self.hist[key][-1]))
        if self.recording:
            self.rec_var.set(f"Đang ghi: {len(self.record_rows)} mẫu, "
                             f"{time.time() - self.record_t0:.1f} s")
        self.after(50, self.poll_queue)

    def update_plot(self):
        # Hiển thị tốc độ nhận dữ liệu thực tế
        now = time.time()
        if now - self._rate_t >= 1.0:
            self.rate_var.set(f"{self._rate_count / (now - self._rate_t):.0f} mẫu/s")
            self._rate_count = 0
            self._rate_t = now

        if self.t_hist and self.plot_lines and not self.paused:
            window = float(self.window_var.get())
            t_all = list(self.t_hist)
            t_end = t_all[-1]
            # Chỉ lấy phần dữ liệu trong cửa sổ, giảm mẫu nếu quá nhiều điểm
            start = len(t_all) - min(len(t_all), int(window * TELEMETRY_HZ) + 1)
            step = max(1, (len(t_all) - start) // PLOT_MAX_POINTS)
            t = t_all[start::step]
            for key, (ax, line) in self.plot_lines.items():
                y = list(self.hist[key])[start::step]
                line.set_data(t, y)
                if y:
                    lo, hi = min(y), max(y)
                    pad = (hi - lo) * 0.1 or max(abs(hi) * 0.1, 1.0)
                    ax.set_ylim(lo - pad, hi + pad)
            first_ax = next(iter(self.plot_lines.values()))[0]
            first_ax.set_xlim(max(0.0, t_end - window), max(t_end, window))
            self.canvas.draw_idle()
        self.after(PLOT_INTERVAL_MS, self.update_plot)

    def on_close(self):
        if self.recording and messagebox.askyesno("Thoát", "Đang ghi dữ liệu. Lưu file trước khi thoát?"):
            self.toggle_record()
        self.recording = False
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    DynoApp().mainloop()
