#!/usr/bin/env python3
"""通过 Wi-Fi 读取 ESP32 状态并发送控制指令。"""
import argparse
from pathlib import Path
import queue
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'jy901p'))
from wifi_client import DEFAULT_ADDRESS, DEFAULT_PORT, WifiClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default=DEFAULT_ADDRESS)
    parser.add_argument('--udp-port', type=int, default=DEFAULT_PORT)
    args = parser.parse_args()
    client = WifiClient(args.ip, args.udp_port)
    pending = queue.Queue()

    def read_input():
        # 终端输入会阻塞。另用线程等待输入，由主循环统一处理 UDP。
        for line in sys.stdin:
            pending.put(line.strip())

    threading.Thread(target=read_input, daemon=True).start()
    print('Wi-Fi 数据：100 Hz 接收、10 Hz 打印。可输入 status / zero / limits / gains / balance / stop。')
    print('输入 quit 或 Ctrl+C 退出；退出和断线不会自动停机。')
    next_print = 0.
    last_event = ''
    try:
        while True:
            sample, _ = client.poll()
            # 持续收包，终端按较低频率打印，便于输入命令。
            if sample and time.monotonic() >= next_print:
                roll, pitch, yaw = sample['angles_deg']
                print(f'Roll={roll:+.3f} Pitch={pitch:+.3f} Yaw={yaw:+.3f} | '
                      f"tilt={sample['tilt_deg']:+.3f}deg mode={sample['mode']} "
                      f"speed={sample['speed_turns_s']:+.3f}turn/s torque={sample['torque_nm']:+.4f}Nm", flush=True)
                next_print = time.monotonic() + .1
            if client.last_event != last_event:
                print(client.last_event, flush=True)
                last_event = client.last_event
            while not pending.empty():
                command = pending.get_nowait()
                if command in ('quit', 'exit'):
                    return 0
                try:
                    client.send_command(command)
                except (OSError, ValueError) as error:
                    print(error, flush=True)
            time.sleep(.01)
    except KeyboardInterrupt:
        return 0
    finally:
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
