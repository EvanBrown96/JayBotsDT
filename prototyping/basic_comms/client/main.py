import socket

PORT = 10001
sock = socket.socket()
sock.bind(('', PORT))
sock.listen()

while True:
    conn, addr = sock.accept()

    while True:
        data = conn.recv(1024)
        if not data:
            break
        print(data)
