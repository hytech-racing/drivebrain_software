import tkinter as tk
from tkinter import ttk
import cantools
import can
import threading
import time

DBC_FILE = "config/hytech.dbc"
CAN_INTERFACE = "vcan1"

# Load CAN
bus = can.Bus(channel=CAN_INTERFACE, interface="socketcan")
db = cantools.database.load_file(DBC_FILE)


class MessageFrame(tk.LabelFrame):
    def __init__(self, master, message, *args, **kwargs):
        super().__init__(master, text=message.name, *args, **kwargs)
        self.message = message
        self.entries = {}
        self.build_fields()

    def build_fields(self):
        for i, signal in enumerate(self.message.signals):
            ttk.Label(self, text=signal.name).grid(row=i, column=0, sticky="e", padx=2, pady=2)
            entry = ttk.Entry(self)
            entry.grid(row=i, column=1, padx=2, pady=2)
            self.entries[signal.name] = entry

    def get_encoded_message(self):
        try:
            data = {name: float(entry.get()) for name, entry in self.entries.items()}
            encoded = self.message.encode(data)
            return can.Message(arbitration_id=self.message.frame_id, is_extended_id=False, data=encoded)
        except Exception as e:
            print(f"[{self.message.name}] Encoding error:", e)
            return None

class ScrollableFrame(ttk.Frame):
    def __init__(self, container, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        canvas = tk.Canvas(self)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(
                scrollregion=canvas.bbox("all")
            )
        )

        self.canvas = canvas

        self.canvas_frame = canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Fix width resizing
        self.bind("<Configure>", self._resize)

    def _resize(self, event):
        self.canvas.itemconfig(self.canvas_frame, width=event.width)

class ScrollableSelectorFrame(tk.LabelFrame):
    def __init__(self, parent, label="Select Messages", height=500):
        super().__init__(parent, text=label)

        # Canvas and scrollbar
        self.canvas = tk.Canvas(self, height=height)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(
                scrollregion=self.canvas.bbox("all")
            )
        )

        self.canvas_frame = self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        # Resize canvas frame with window
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event):
        # Make the canvas window match the width of the label frame
        self.canvas.itemconfig(self.canvas_frame, width=self.canvas.winfo_width())


class CANMultiSenderGUI(tk.Tk):
    def __init__(self, db):
        super().__init__()
        self.title("CAN Multi-Message Sender")
        self.geometry("900x900")
        self.db = db
        self.frames = {}
        self.check_vars = {}
        self.running = False

        self.build_ui()

    def build_ui(self):
        selector_wrapper = ttk.LabelFrame(self, text="Select Messages")
        selector_wrapper.pack(fill="x", padx=5, pady=5)

        self.selector_frame = ScrollableSelectorFrame(self, label="Select Messages", height=400)
        self.selector_frame.pack(fill="x", padx=5, pady=5)

        for msg in self.db.messages:
            var = tk.BooleanVar()
            cb = ttk.Checkbutton(
                self.selector_frame.scrollable_frame,
                text=msg.name,
                variable=var,
                command=self.refresh_messages
            )
            cb.pack(anchor="w")
            self.check_vars[msg.name] = var

        # Scrollable message container
        self.scrollable_message_area = ScrollableFrame(self)
        self.scrollable_message_area.pack(fill="both", expand=True, padx=5, pady=5)

        # Start/Stop button
        self.toggle_btn = ttk.Button(self, text="Start Sending", command=self.toggle_sending)
        self.toggle_btn.pack(pady=10)

    def refresh_messages(self):
        # Clear all frames
        for widget in self.scrollable_message_area.scrollable_frame.winfo_children():
            widget.destroy()
        self.frames.clear()

        for name, var in self.check_vars.items():
            if var.get():
                msg = self.db.get_message_by_name(name)
                frame = MessageFrame(self.scrollable_message_area.scrollable_frame, msg)
                frame.pack(fill="x", padx=5, pady=5)
                self.frames[name] = frame

    def toggle_sending(self):
        self.running = not self.running
        if self.running:
            self.toggle_btn.config(text="Stop Sending")
            threading.Thread(target=self.send_loop, daemon=True).start()
        else:
            self.toggle_btn.config(text="Start Sending")

    def send_loop(self):
        while self.running:
            for msg_name, frame in self.frames.items():
                message = frame.get_encoded_message()
                if message:
                    try:
                        bus.send(message)
                    except Exception as e:
                        print(f"Send error for {msg_name}:", e)
            time.sleep(0.004)

if __name__ == "__main__":
    app = CANMultiSenderGUI(db)
    app.mainloop()