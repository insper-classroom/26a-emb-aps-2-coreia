#!/usr/bin/env python3

import sys
import glob
import serial
import pyautogui
pyautogui.PAUSE = 0  # Remove delay between actions
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox


# Protocolo binário: [0xFF, axis, value + 128]
# axis:  0 = X, 1 = Y, 2 = CLICK
# value: int em [-95, +95] deslocado para [33, 223]
SYNC_BYTE    = 0xFF
VALUE_OFFSET = 128
AXIS_X       = 0
AXIS_Y       = 1
AXIS_CLICK   = 2          # novo: evento de clique enviado pelo firmware
AXIS_ROT_R   = 3          # botao: girar direita -> seta cima
AXIS_ROT_L   = 4          # botao: girar esquerda -> Z
MAX_ABS_VALUE = 95

# Coloca em True para imprimir cada pacote decodificado no terminal
DEBUG = False


def move_mouse(axis, value):
    """Move o mouse de acordo com o eixo e valor recebidos."""
    if axis == AXIS_X:
        pyautogui.moveRel(int(value / 10), 0)
    elif axis == AXIS_Y:
        pyautogui.moveRel(0, int(value / 10))


def parse_data(data):
    """
    Interpreta 2 bytes de payload: [axis, value+128].
    Retorna (axis, value) com value já de volta em [-95, +95].
    """
    axis  = data[0]
    value = data[1] - VALUE_OFFSET
    return axis, value


def controle(ser):
    """
    Loop principal.
    Protocolo: 0xFF (sync) + 2 bytes [axis, value+128].
    axis = 2 indica um clique do mouse.
    """
    while True:
        # ── Sincronização ──────────────────────────────────────────────────────
        sync_byte = ser.read(size=1)
        if not sync_byte:
            continue
        if sync_byte[0] != SYNC_BYTE:
            continue

        data = ser.read(size=2)
        if len(data) < 2:
            continue

        # Se o primeiro byte do payload for 0xFF é na verdade o próximo header — resync
        if data[0] == SYNC_BYTE:
            continue

        axis, value = parse_data(data)

        # ── Evento de clique ───────────────────────────────────────────────────
        if axis == AXIS_CLICK:
            pyautogui.click()
            if DEBUG:
                print("CLICK!")
            continue

        # ── Rotacao (teclado) ──────────────────────────────────────────────────
        if axis == AXIS_ROT_R:
            pyautogui.press('up')
            if DEBUG:
                print("ROT R (up)")
            continue
        if axis == AXIS_ROT_L:
            pyautogui.press('z')
            if DEBUG:
                print("ROT L (z)")
            continue

        # ── Movimento do mouse ─────────────────────────────────────────────────
        if axis not in (AXIS_X, AXIS_Y):
            continue
        if value < -MAX_ABS_VALUE or value > MAX_ABS_VALUE:
            continue

        if DEBUG:
            print(f"axis={axis} value={value}")

        move_mouse(axis, value)


# ==============================================================================
#  Interface gráfica e utilitários (inalterados em relação ao lab anterior)
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