import socket
import json
import cv2
import base64
import numpy as np
import io, os

class Server:
    def __init__(self, address='0.0.0.0', port=2000):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
        self.socket.bind((address, port))
        self.buf = io.BytesIO()
        self.num_lines = 0
        self.pointer = self.buf.tell()

        self.logfile = open('ok_lost_msg.txt', 'w')

    def __del__(self):
        self.close()

    def close(self):
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
        except:
            pass
        finally:
            self.logfile.close()

    def wait_for_connection(self):
        self.socket.listen(1)
        self.conn, _ = self.socket.accept()

    def wait_for_data(self):
        self.pointer = self.buf.tell()
        while self.num_lines == 0:
            byte_data = self.conn.recv(4096)
            self.num_lines = byte_data.count(b'\n')
            self.buf.seek(0, 2)
            self.buf.write(byte_data)
        self.num_lines -= 1
        self.buf.seek(self.pointer)
        line = self.buf.readline().decode()
        data = json.loads(line)
        #if 'pose' in data:
       	#    print('yaw =', data['pose'][3] * 180 / np.pi)
        self.logfile.write(line)
        if 'op' not in data:
            print(data)
            return
        if data['op'] == 'publish':
            if data['topic'].startswith('/image'):
                b64_str = data['msg']['data']

                img_enc = np.frombuffer(base64.b64decode(b64_str), dtype=np.uint8)
                img = cv2.imdecode(img_enc, flags=cv2.IMREAD_COLOR)
                return img

            else:
                print(data)
            
        #imgs = []
        #for i in range(4):
        #    b64_str = data.get(f'image{i+1}', None)
        #    if b64_str is not None:
        #        img_enc = np.frombuffer(base64.b64decode(b64_str), dtype=np.uint8)
        #        img = cv2.imdecode(img_enc, flags=cv2.IMREAD_COLOR)
        #    else:
        #        img = np.zeros([480,640,3], dtype=np.uint8)
        #    imgs.append(img)
        #return imgs

server = Server()

print("Waiting for a connection from KakiP...")
server.wait_for_connection()
print("Connected.")

while True:
    imgs = server.wait_for_data()
    #imgt = np.concatenate(imgs[:2], axis=1)
    #imgb = np.concatenate(imgs[2:], axis=1)
    #img = np.concatenate([imgt, imgb], axis=0)
    #cv2.imshow('KakiP', img)
    if imgs is not None:
        cv2.imshow('KakiP', imgs)
    key = cv2.waitKey(100)
