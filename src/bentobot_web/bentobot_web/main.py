"""bentobot_web entry point: rclpy in a background thread, aiohttp on asyncio."""
import asyncio
import fcntl
import os
import signal
import socket
import struct
import threading

import rclpy
from aiohttp import web
from rclpy.executors import SingleThreadedExecutor

from .bluetooth import Bluetooth
from .controllers import ControllerProfiles
from .maps import MapStore
from .modes import ModeManager
from .ros_iface import WebNode
from .system import SystemStats
from .web import WebApp


def iface_address(iface):
    """IP to bind for an interface name (wlan0) or a literal address (0.0.0.0)."""
    try:
        socket.inet_aton(iface)
        return iface
    except OSError:
        pass
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', iface[:15].encode()))  # SIOCGIFADDR
        return socket.inet_ntoa(packed[20:24])
    except OSError:
        return None
    finally:
        s.close()


async def serve(node, stop_event):
    log = node.get_logger()
    data_dir = node.p_data_dir
    os.makedirs(data_dir, exist_ok=True)

    modes = ModeManager(os.path.join(data_dir, '.mode.pgid'), logger=log)
    app = WebApp(
        node=node,
        modes=modes,
        store=MapStore(os.path.join(data_dir, 'maps')),
        profiles=ControllerProfiles(os.path.join(data_dir, 'controllers.json')),
        bt=Bluetooth(logger=log),
        stats=SystemStats(data_dir),
        logger=log,
    )
    await app.start()

    runner = web.AppRunner(app.app, access_log=None)
    await runner.setup()

    # wait for the interface to get an address (Wi-Fi can come up after the container)
    host = None
    while host is None and not stop_event.is_set():
        host = iface_address(node.p_iface)
        if host is None:
            log.warn('interface %s has no IPv4 address yet; retrying in 5 s '
                     '(set BENTOBOT_WEB_IFACE, e.g. eth0 or 0.0.0.0)' % node.p_iface)
            try:
                await asyncio.wait_for(stop_event.wait(), 5.0)
            except asyncio.TimeoutError:
                pass
    if host is not None:
        site = web.TCPSite(runner, host, node.p_port)
        await site.start()
        log.info('web UI on http://%s:%d/ (%s)' % (host, node.p_port, node.p_iface))

    await stop_event.wait()
    log.info('shutting down')
    await asyncio.get_running_loop().run_in_executor(None, modes.shutdown)
    await app.stop()
    await runner.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)
    try:
        loop.run_until_complete(serve(node, stop_event))
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
        loop.close()


if __name__ == '__main__':
    main()
