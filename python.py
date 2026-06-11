#!/usr/bin/env python3

import sys
import glob
import time
import serial
from pynput.mouse import Controller as MouseCtl
from pynput.keyboard import Controller as KbCtl, Key
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox


# Protocolo binário: [0xFF, axis, value + 128]
# axis:  0 = X, 1 = Y, 2 = CLICK (ignored by this receiver)
# value: int em [-95, +95] deslocado para [33, 223]
SYNC_BYTE    = 0xFF
VALUE_OFFSET = 128
AXIS_X       = 0
AXIS_Y       = 1
AXIS_CLICK   = 2          # evento de clique do acelerometro, ignorado
AXIS_ROT_R   = 3          # botao: girar direita -> seta cima
AXIS_SPACE   = 5          # evento enviado apenas pela deteccao da IA
AXIS_AI_CONFIDENCE = 6    # confianca da classe "tetris", escalada para 0..94
AXIS_AI_STATUS = 7
AXIS_AI_PROGRESS = 8
AXIS_DEBUG_ACCEL_X = 9
AXIS_DEBUG_ACCEL_Y = 10
AXIS_DEBUG_ACCEL_Z = 11
AXIS_AI_IDLE_CONFIDENCE = 12
AXIS_AI_ANOMALY = 13
AXIS_AI_DSP_MS = 14
AXIS_AI_CLASSIFICATION_MS = 15
AXIS_AI_QUEUE_DEPTH = 16
AXIS_AI_ERROR_CODE = 17
AXIS_IMU_READ_ERRORS = 18
AXIS_ENTER = 19
AXIS_ESC = 20
AXIS_AUDIO_STATUS = 21
AXIS_AUDIO_INDEX_LOW = 22
AXIS_AUDIO_INDEX_MID = 23
AXIS_AUDIO_INDEX_HIGH = 24
MAX_ABS_VALUE = 95
WAV_DATA_LENGTH = 742926
AUDIO_SAMPLE_RATE = 11000

AUDIO_STATUS = {
    1: "started",
    2: "paused",
    3: "resumed",
}

AI_STATUS = {
    1: "AI task started",
    2: "pause active; collecting IMU samples",
    3: "AI window ready; running classifier",
    4: "classifier error",
    5: "tetris detected",
    6: "pause released; AI buffers reset",
}

# Coloca em True para imprimir cada pacote decodificado no terminal
DEBUG = True


mouse = MouseCtl()
kb = KbCtl()
debug_start = time.monotonic()


def debug_print(message):
    elapsed = time.monotonic() - debug_start
    print(f"[{elapsed:8.3f}s] {message}", flush=True)


def controle(ser):
    """
    Loop principal.
    Protocolo: 0xFF (sync) + 2 bytes [axis, value+128].

    Drena tudo que chegou na serial a cada iteracao e aplica apenas o ULTIMO
    estado de X/Y (um unico mouse.move por ciclo). Botoes sao eventos discretos
    e executam todos. O clique do acelerometro (axis 2) e ignorado. Usa pynput
    (chamadas em microssegundos) em vez
    de pyautogui (dezenas de ms por chamada), que nao acompanhava 200 pacotes/s.

    O pause do IMU e resolvido inteiro no firmware: enquanto pausado o Pico
    para de enviar X/Y/click, e o cursor congela porque so anda quando chega
    pacote. Nenhum tratamento e necessario aqui.
    """
    ser.timeout = 0
    buf = bytearray()
    last = {AXIS_X: 0, AXIS_Y: 0}
    audio_index_parts = [0, 0, 0]
    previous_audio_index = None
    while True:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk

        moved = False
        while len(buf) >= 3:
            if buf[0] != SYNC_BYTE:
                del buf[0]
                continue
            # Se o payload começa com 0xFF é na verdade o próximo header — resync
            if buf[1] == SYNC_BYTE:
                del buf[0]
                continue

            axis  = buf[1]
            value = buf[2] - VALUE_OFFSET
            del buf[:3]

            if axis == AXIS_CLICK:
                continue
            elif axis == AXIS_ROT_R:
                kb.tap(Key.up)
                if DEBUG:
                    print("ROT R (up)")
            elif axis == AXIS_SPACE:
                kb.tap(Key.space)
                if DEBUG:
                    print("SPACE (AI)")
            elif axis == AXIS_ENTER:
                kb.tap(Key.enter)
                if DEBUG:
                    print("ENTER")
            elif axis == AXIS_ESC:
                kb.tap(Key.esc)
                if DEBUG:
                    print("ESC")
            elif axis == AXIS_AUDIO_STATUS:
                debug_print(f"Audio status: {AUDIO_STATUS.get(value, f'unknown ({value})')}")
            elif axis == AXIS_AUDIO_INDEX_LOW:
                audio_index_parts[0] = value
            elif axis == AXIS_AUDIO_INDEX_MID:
                audio_index_parts[1] = value
            elif axis == AXIS_AUDIO_INDEX_HIGH:
                audio_index_parts[2] = value
                audio_index = (
                    audio_index_parts[0]
                    + audio_index_parts[1] * 95
                    + audio_index_parts[2] * 95 * 95
                )
                delta = (
                    "first report"
                    if previous_audio_index is None
                    else f"delta={audio_index - previous_audio_index:+d}"
                )
                debug_print(
                    f"Audio index: {audio_index}/{WAV_DATA_LENGTH} "
                    f"({audio_index / AUDIO_SAMPLE_RATE:.2f}s, {delta})"
                )
                previous_audio_index = audio_index
            elif axis == AXIS_AI_CONFIDENCE:
                debug_print(f"AI tetris confidence: {value * 100 / 94:.1f}%")
            elif axis == AXIS_AI_STATUS:
                debug_print(f"AI status: {AI_STATUS.get(value, f'unknown ({value})')}")
            elif axis == AXIS_AI_PROGRESS:
                debug_print(f"AI window collection: {value * 100 / 94:.1f}%")
            elif axis == AXIS_DEBUG_ACCEL_X:
                debug_print(f"IMU accel X: {value / 20:.2f} g")
            elif axis == AXIS_DEBUG_ACCEL_Y:
                debug_print(f"IMU accel Y: {value / 20:.2f} g")
            elif axis == AXIS_DEBUG_ACCEL_Z:
                debug_print(f"IMU accel Z: {value / 20:.2f} g")
            elif axis == AXIS_AI_IDLE_CONFIDENCE:
                debug_print(f"AI idle confidence: {value * 100 / 94:.1f}%")
            elif axis == AXIS_AI_ANOMALY:
                debug_print(f"AI anomaly score: {value / 10:.1f}")
            elif axis == AXIS_AI_DSP_MS:
                debug_print(f"AI DSP time: {value} ms")
            elif axis == AXIS_AI_CLASSIFICATION_MS:
                debug_print(f"AI classification time: {value} ms")
            elif axis == AXIS_AI_QUEUE_DEPTH:
                debug_print(f"AI sample queue depth: {value}")
            elif axis == AXIS_AI_ERROR_CODE:
                debug_print(f"AI classifier error code: {value}")
            elif axis == AXIS_IMU_READ_ERRORS:
                debug_print(f"IMU read errors: {value}")
            elif axis in (AXIS_X, AXIS_Y) and abs(value) <= MAX_ABS_VALUE:
                last[axis] = value
                moved = True

        if moved:
            mouse.move(int(last[AXIS_X] / 10), int(last[AXIS_Y] / 10))

        time.sleep(0.001)


# ==============================================================================
#  Interface gráfica e utilitários
# ==============================================================================

def serial_ports():
    """Retorna uma lista das portas seriais disponíveis na máquina."""
    ports = []
    if sys.platform.startswith('win'):
        for i in range(1, 256):
            port = f'COM{i}'
            try:
                s = serial.Serial(port)
                s.close()
                ports.append(port)
            except (OSError, serial.SerialException):
                pass
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Plataforma não suportada para detecção de portas seriais.')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def conectar_porta(port_name, root, botao_conectar, status_label, mudar_cor_circulo):
    """Abre a conexão com a porta selecionada e inicia o loop de leitura."""
    if not port_name:
        messagebox.showwarning("Aviso", "Selecione uma porta serial antes de conectar.")
        return

    ser = None
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        status_label.config(text=f"Conectado em {port_name}", foreground="green")
        mudar_cor_circulo("green")
        botao_conectar.config(text="Conectado")
        root.update()

        controle(ser)

    except KeyboardInterrupt:
        print("Encerrando via KeyboardInterrupt.")
    except Exception as e:
        messagebox.showerror("Erro de Conexão", f"Não foi possível conectar em {port_name}.\nErro: {e}")
        mudar_cor_circulo("red")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
        status_label.config(text="Conexão encerrada.", foreground="red")
        mudar_cor_circulo("red")


def criar_janela():
    root = tk.Tk()
    root.title("Controle de Mouse")
    root.geometry("400x250")
    root.resizable(False, False)

    dark_bg    = "#2e2e2e"
    dark_fg    = "#ffffff"
    accent_color = "#007acc"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame",  background=dark_bg)
    style.configure("TLabel",  background=dark_bg, foreground=dark_fg, font=("Segoe UI", 11))
    style.configure("TButton", font=("Segoe UI", 10, "bold"),
                    foreground=dark_fg, background="#444444", borderwidth=0)
    style.map("TButton", background=[("active", "#555555")])
    style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"),
                    foreground=dark_fg, background=accent_color, padding=6)
    style.map("Accent.TButton", background=[("active", "#005f9e")])
    style.configure("TCombobox",
                    fieldbackground=dark_bg, background=dark_bg,
                    foreground=dark_fg, padding=4)
    style.map("TCombobox", fieldbackground=[("readonly", dark_bg)])

    frame_principal = ttk.Frame(root, padding="20")
    frame_principal.pack(expand=True, fill="both")

    titulo_label = ttk.Label(frame_principal,
                             text="Controle de Mouse",
                             font=("Segoe UI", 14, "bold"))
    titulo_label.pack(pady=(0, 10))

    porta_var = tk.StringVar(value="")

    botao_conectar = ttk.Button(
        frame_principal,
        text="Conectar e Iniciar Leitura",
        style="Accent.TButton",
        command=lambda: conectar_porta(
            porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo
        )
    )
    botao_conectar.pack(pady=10)

    footer_frame = tk.Frame(root, bg=dark_bg)
    footer_frame.pack(side="bottom", fill="x", padx=10, pady=(10, 0))

    status_label = tk.Label(footer_frame,
                            text="Aguardando seleção de porta...",
                            font=("Segoe UI", 11), bg=dark_bg, fg=dark_fg)
    status_label.grid(row=0, column=0, sticky="w")

    portas_disponiveis = serial_ports()
    if portas_disponiveis:
        porta_var.set(portas_disponiveis[0])
    port_dropdown = ttk.Combobox(footer_frame, textvariable=porta_var,
                                 values=portas_disponiveis,
                                 state="readonly", width=10)
    port_dropdown.grid(row=0, column=1, padx=10)

    circle_canvas = tk.Canvas(footer_frame, width=20, height=20,
                              highlightthickness=0, bg=dark_bg)
    circle_item = circle_canvas.create_oval(2, 2, 18, 18, fill="red", outline="")
    circle_canvas.grid(row=0, column=2, sticky="e")

    footer_frame.columnconfigure(1, weight=1)

    def mudar_cor_circulo(cor):
        circle_canvas.itemconfig(circle_item, fill=cor)

    root.mainloop()


if __name__ == "__main__":
    criar_janela()
