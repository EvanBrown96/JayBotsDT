import socket
import tkinter as tk
import tkinter.ttk as ttk

# setup socket
PORT = 10001
sock = socket.socket()
sock.bind(('', PORT))
sock.listen()

conn_state = False
conn = None

# setup GUI
master = tk.Tk()

conn_text = tk.Label(master, text="No Connection")
conn_text.grid(row=0, column=0)

divider = ttk.Separator(master, orient='horizontal')
divider.grid(row=1, column=0, sticky=tk.W+tk.E)

CANVAS_SIZE = 200
canvas = tk.Canvas(master, height=CANVAS_SIZE, width=CANVAS_SIZE)
canvas.grid(row=2, column=0)


canvas.create_line(CANVAS_SIZE/2, 0, CANVAS_SIZE/2, CANVAS_SIZE)
canvas.create_line(0, CANVAS_SIZE/2, CANVAS_SIZE, CANVAS_SIZE/2)
pointer_id = canvas.create_oval(CANVAS_SIZE/2-10, CANVAS_SIZE/2-10, CANVAS_SIZE/2+10, CANVAS_SIZE/2+10, fill='black')


def update_pos(event):
    canvas.coords(pointer_id, round(event.x-10, -1), round(event.y-10, -1), round(event.x+10, -1), round(event.y+10, -1))
    send = "{},{}".format(-1*(event.y-CANVAS_SIZE/2), event.x-CANVAS_SIZE/2)
    try:
        conn.send(send.encode('utf-8'))
    except ConnectionError:
        conn_state = False
        conn_text.configure(text='No connection')


canvas.bind('<Button-1>', update_pos)


def check_state():
    global conn_state, conn
    if conn_state:
        try:
            conn.send(b"persist")
        except ConnectionError:
            conn_state = False
            conn_text.configure(text='No connection')
    else:
        conn, addr = sock.accept()
        conn_state = True
        conn_text.configure(text='Connected to {}'.format(addr))

    master.after(1000, check_state)

master.after(1000, check_state)


master.mainloop()
#
# while True:
#
#     conn, addr = sock.accept()
#     print("Connected to rover.")
#
#     conn.send(b"Connected to Server")
#
#     conn.close()