import socket
import tkinter as tk
import tkinter.ttk as ttk

# setup socket
PORT = 10001
addr = input("Enter IP Address of rover")
sock = socket.socket()

conn_state = False

# setup GUI
master = tk.Tk()

conn_text = tk.Label(master, text="No Connection")
conn_text.grid(row=0, column=0)

divider = ttk.Separator(master, orient='horizontal')
divider.grid(row=1, column=0, sticky=tk.W+tk.E)

CANVAS_SIZE = 400
MIDDLE = CANVAS_SIZE/2
canvas = tk.Canvas(master, height=CANVAS_SIZE, width=CANVAS_SIZE)
canvas.grid(row=2, column=0)


canvas.create_line(MIDDLE, 0, MIDDLE, CANVAS_SIZE)
canvas.create_line(0, MIDDLE, CANVAS_SIZE, MIDDLE)
canvas.create_polygon([0, 0, MIDDLE, 0, 0, MIDDLE, 0, 0], fill='black')
canvas.create_polygon([CANVAS_SIZE, 0, CANVAS_SIZE, MIDDLE, MIDDLE, 0, CANVAS_SIZE, 0], fill='black')
canvas.create_polygon([CANVAS_SIZE, CANVAS_SIZE, CANVAS_SIZE, MIDDLE, MIDDLE, CANVAS_SIZE, CANVAS_SIZE, CANVAS_SIZE], fill='black')
canvas.create_polygon([0, CANVAS_SIZE, MIDDLE, CANVAS_SIZE, 0, MIDDLE, 0, CANVAS_SIZE], fill='black')
pointer_id = canvas.create_oval(MIDDLE-10, MIDDLE-10, MIDDLE+10, MIDDLE+10, fill='black')


def update_pos(event):
    global conn_state, sock
    raw_x = round(event.x, -1)
    raw_y = round(event.y, -1)

    canvas.coords(pointer_id, raw_x-10, raw_y-10, raw_x+10, raw_y+10)
    msg = "{},{}".format(raw_x-MIDDLE, -1*(raw_y-MIDDLE))
    try:
        sock.send(msg.encode('utf-8'))
    except ConnectionError:
        conn_state = False
        conn_text.configure(text='No connection')


canvas.bind('<Button-1>', update_pos)


def check_state():
    global conn_state, sock
    if conn_state:
        try:
            sock.send(b"persist")
        except ConnectionError:
            conn_state = False
            conn_text.configure(text='No connection')
    else:
        try:
            sock.connect((addr, PORT))
        except Exception:
            pass
        else:
            conn_state = True
            conn_text.configure(text='Connected to {}'.format(addr))

    master.after(1000, check_state)


master.after(0, check_state)

master.mainloop()