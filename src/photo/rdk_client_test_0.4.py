import zmq
import cv2
import time
import os
import sys
import uuid

SERVER_IP = os.getenv("SERVER_IP", "172.20.10.2")
SERVER_ENDPOINT = f"tcp://{SERVER_IP}:5555"
IMAGE_PATH = os.getenv("IMAGE_PATH", "/userdata/dev_ws/src/photo/my_photo.png")

UPLOAD_TIMEOUT = 10000
POLL_TIMEOUT = 5000
REQUEST_RETRIES = 10
RESULT_POLL_INITIAL_DELAY = 4.0
RESULT_POLL_INTERVAL = 1.0
RESULT_POLL_MAX_INTERVAL = 5.0
RESULT_POLL_RETRIES = 180

def send_image_and_exit():
    if not os.path.exists(IMAGE_PATH):
        print("[-] 找不到图片")
        sys.exit(1)

    img = cv2.imread(IMAGE_PATH)
    img = cv2.resize(img, (640, 400), interpolation=cv2.INTER_AREA)
    _, img_encoded = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    image_bytes = img_encoded.tobytes()

    context = zmq.Context()
    retries_left = REQUEST_RETRIES
    request_id = uuid.uuid4().hex.encode("ascii")

    upload_ok = False
    while retries_left > 0:
        client = context.socket(zmq.REQ)
        client.setsockopt(zmq.LINGER, 0)
        client.connect(SERVER_ENDPOINT)

        client.send_multipart([request_id, b"UPLOAD", image_bytes])

        poller = zmq.Poller()
        poller.register(client, zmq.POLLIN)

        if poller.poll(UPLOAD_TIMEOUT):
            reply_parts = client.recv_multipart()
            if len(reply_parts) != 3:
                print("[-] 收到异常回包格式")
                client.close()
                context.term()
                sys.exit(2)

            reply_id, status, text = reply_parts
            if reply_id != request_id:
                print("[-] 回包ID不匹配")
                client.close()
                context.term()
                sys.exit(2)

            if status == b"ACK":
                print("\n[+] 图片已被PC接收，等待推理结果...")
                client.close()
                upload_ok = True
                break

            if status == b"OK":
                print("\n[+] 收到推理结果:\n" + text.decode("utf-8", "replace"))
                client.close()
                context.term()
                sys.exit(0)

            print("\n[-] 推理失败:\n" + text.decode("utf-8", "replace"))
            client.close()
            context.term()
            sys.exit(2)
        else:
            print(f"没收到回执，重试... (剩余 {retries_left-1} 次)")
            client.close()
            poller.unregister(client)
            
            retries_left -= 1
            time.sleep(0.1) 

    if not upload_ok:
        print("\n[✖] 尝试了 10 次都没传过去，失败了")
        context.term()
        sys.exit(1)

    poll_retries = RESULT_POLL_RETRIES
    poll_interval = RESULT_POLL_INTERVAL
    if RESULT_POLL_INITIAL_DELAY > 0:
        time.sleep(RESULT_POLL_INITIAL_DELAY)
        
    while poll_retries > 0:
        client = context.socket(zmq.REQ)
        client.setsockopt(zmq.LINGER, 0)
        client.connect(SERVER_ENDPOINT)

        client.send_multipart([request_id, b"GET"])

        poller = zmq.Poller()
        poller.register(client, zmq.POLLIN)

        if poller.poll(POLL_TIMEOUT):
            reply_parts = client.recv_multipart()
            if len(reply_parts) != 3:
                print("[-] 收到异常回包格式")
                client.close()
                context.term()
                sys.exit(2)

            reply_id, status, text = reply_parts
            if reply_id != request_id:
                print("[-] 回包ID不匹配")
                client.close()
                context.term()
                sys.exit(2)

            if status == b"PENDING":
                client.close()
                poll_retries -= 1
                time.sleep(poll_interval)
                if poll_interval < RESULT_POLL_MAX_INTERVAL:
                    poll_interval = min(poll_interval * 2.0, RESULT_POLL_MAX_INTERVAL)
                continue

            if status == b"OK":
                print("\n[+] 收到推理结果:\n" + text.decode("utf-8", "replace"))
                client.close()
                context.term()
                sys.exit(0)

            print("\n[-] 推理失败:\n" + text.decode("utf-8", "replace"))
            client.close()
            context.term()
            sys.exit(2)
        else:
            print("[-] 获取结果超时，继续等待...")
            client.close()
            poller.unregister(client)
            poll_retries -= 1
            time.sleep(poll_interval)
            if poll_interval < RESULT_POLL_MAX_INTERVAL:
                poll_interval = min(poll_interval * 2.0, RESULT_POLL_MAX_INTERVAL)

    print("\n[✖] 等待推理结果超时")
    context.term()
    sys.exit(1)

if __name__ == "__main__":
    send_image_and_exit()