import time
import redis

KEY = "action_neck_unitree_g1_with_hands"

pc = redis.Redis(host="127.0.0.1", port=6379, db=0, decode_responses=False)
g1 = redis.Redis(host="192.168.123.164", port=6379, db=0, decode_responses=False)

last = object()

while True:
    v = pc.get(KEY)
    if v is not None and v != last:
        g1.set(KEY, v)
        last = v
    time.sleep(0.01)
