import socket

ip = socket.gethostbyname('benkpress.campus.ltu.se')
print (ip)

HOST = '130.240.207.52'  
PORT = 1235 

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            conn.sendall(data)
