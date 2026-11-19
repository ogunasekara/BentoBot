"""BlueZ (host bluetoothd) over the system D-Bus, using dbus-next.

The container needs /var/run/dbus bind-mounted; it runs as root, which the
default BlueZ D-Bus policy allows. A NoInputNoOutput agent is registered so
gamepads pair with "just works" pairing and no PIN prompts.
"""
import asyncio
import os

try:
    from dbus_next import BusType, Message, MessageType, Variant
    from dbus_next.aio import MessageBus
    from dbus_next.service import ServiceInterface, method
except ImportError:  # pragma: no cover
    MessageBus = None
    ServiceInterface = object

    def method():
        return lambda f: f

BLUEZ = 'org.bluez'
AGENT_PATH = '/bentobot/agent'
DISCOVERY_SECONDS = 30
SOCKET = '/var/run/dbus/system_bus_socket'


class BluetoothError(Exception):
    pass


class Agent(ServiceInterface):
    """Accepts every pairing request (trusted LAN, headless robot)."""

    def __init__(self):
        super().__init__('org.bluez.Agent1')

    @method()
    def Release(self):
        pass

    @method()
    def RequestPinCode(self, device: 'o') -> 's':  # noqa: F821
        return '0000'

    @method()
    def DisplayPinCode(self, device: 'o', pincode: 's'):  # noqa: F821
        pass

    @method()
    def RequestPasskey(self, device: 'o') -> 'u':  # noqa: F821
        return 0

    @method()
    def DisplayPasskey(self, device: 'o', passkey: 'u', entered: 'q'):  # noqa: F821
        pass

    @method()
    def RequestConfirmation(self, device: 'o', passkey: 'u'):  # noqa: F821
        pass

    @method()
    def RequestAuthorization(self, device: 'o'):  # noqa: F821
        pass

    @method()
    def AuthorizeService(self, device: 'o', uuid: 's'):  # noqa: F821
        pass

    @method()
    def Cancel(self):
        pass


def _unwrap(props):
    return {k: (v.value if isinstance(v, Variant) else v) for k, v in props.items()}


class Bluetooth:
    def __init__(self, logger=None):
        self.logger = logger
        self.bus = None
        self.agent = None
        self.error = None
        self.busy = {}  # address -> action in progress
        self._discovery_task = None
        self._lock = asyncio.Lock()

    async def _ensure_bus(self):
        if self.bus is not None and self.bus.connected:
            return self.bus
        if MessageBus is None:
            raise BluetoothError('dbus-next is not installed')
        if not os.path.exists(SOCKET):
            raise BluetoothError('%s not found: bind-mount /var/run/dbus into the container' % SOCKET)
        try:
            self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        except Exception as e:
            raise BluetoothError('cannot connect to the system D-Bus: %s' % e)
        self.agent = None
        return self.bus

    async def _call(self, path, interface, member, signature='', body=None, timeout=10.0,
                    destination=BLUEZ):
        bus = await self._ensure_bus()
        msg = Message(destination=destination, path=path, interface=interface, member=member,
                      signature=signature, body=body or [])
        try:
            reply = await asyncio.wait_for(bus.call(msg), timeout)
        except asyncio.TimeoutError:
            raise BluetoothError('%s timed out' % member)
        if reply.message_type == MessageType.ERROR:
            text = reply.body[0] if reply.body else ''
            raise BluetoothError('%s: %s %s' % (member, reply.error_name, text))
        return reply.body

    async def _bluez_running(self):
        body = await self._call('/org/freedesktop/DBus', 'org.freedesktop.DBus', 'NameHasOwner',
                                's', [BLUEZ], destination='org.freedesktop.DBus')
        return bool(body[0])

    async def _objects(self):
        body = await self._call('/', 'org.freedesktop.DBus.ObjectManager', 'GetManagedObjects')
        return body[0]

    async def _adapter(self, objects=None):
        objects = objects if objects is not None else await self._objects()
        for path, ifaces in sorted(objects.items()):
            if 'org.bluez.Adapter1' in ifaces:
                return path, _unwrap(ifaces['org.bluez.Adapter1'])
        raise BluetoothError('no Bluetooth adapter found (is the host adapter up? try `rfkill list`)')

    async def _ensure_agent(self):
        bus = await self._ensure_bus()
        if self.agent is None:
            self.agent = Agent()
            bus.export(AGENT_PATH, self.agent)
        try:
            await self._call('/org/bluez', 'org.bluez.AgentManager1', 'RegisterAgent', 'os',
                             [AGENT_PATH, 'NoInputNoOutput'])
        except BluetoothError as e:
            if 'AlreadyExists' not in str(e):
                raise
        try:
            await self._call('/org/bluez', 'org.bluez.AgentManager1', 'RequestDefaultAgent', 'o',
                             [AGENT_PATH])
        except BluetoothError:
            pass  # another default agent (e.g. a desktop) is fine for outgoing pairing

    # ---- public API ----

    async def status(self):
        try:
            if not await self._bluez_running():
                return {'available': False,
                        'error': 'bluetoothd is not running on the host '
                                 '(sudo systemctl enable --now bluetooth)'}
            path, ad = await self._adapter()
        except BluetoothError as e:
            return {'available': False, 'error': str(e)}
        return {
            'available': True,
            'adapter': path,
            'address': ad.get('Address'),
            'name': ad.get('Alias') or ad.get('Name'),
            'powered': bool(ad.get('Powered')),
            'discovering': bool(ad.get('Discovering')),
        }

    async def devices(self):
        objects = await self._objects()
        adapter, _ = await self._adapter(objects)
        out = []
        for path, ifaces in objects.items():
            dev = ifaces.get('org.bluez.Device1')
            if dev is None or not path.startswith(adapter + '/'):
                continue
            d = _unwrap(dev)
            addr = d.get('Address')
            uuids = d.get('UUIDs') or []
            icon = d.get('Icon') or ''
            out.append({
                'address': addr,
                'name': d.get('Name') or d.get('Alias') or addr,
                'alias': d.get('Alias'),
                'paired': bool(d.get('Paired')),
                'trusted': bool(d.get('Trusted')),
                'connected': bool(d.get('Connected')),
                'rssi': d.get('RSSI'),
                'icon': icon,
                # 0x1124 = HID; Icon is set from the class of device
                'gamepad': icon == 'input-gaming' or any(u.startswith('00001124') for u in uuids),
                'busy': self.busy.get(addr),
            })
        out.sort(key=lambda d: (not d['connected'], not d['paired'], not d['gamepad'],
                                -(d['rssi'] if d['rssi'] is not None else -999), d['name'] or ''))
        return out

    async def _device_path(self, address):
        objects = await self._objects()
        adapter, _ = await self._adapter(objects)
        path = '%s/dev_%s' % (adapter, address.upper().replace(':', '_'))
        if path not in objects:
            raise BluetoothError('device %s not found (scan first)' % address)
        return adapter, path

    async def set_powered(self, on):
        adapter, _ = await self._adapter()
        await self._call(adapter, 'org.freedesktop.DBus.Properties', 'Set', 'ssv',
                         ['org.bluez.Adapter1', 'Powered', Variant('b', bool(on))])

    async def scan(self, on):
        adapter, ad = await self._adapter()
        if on:
            if not ad.get('Powered'):
                await self.set_powered(True)
            try:
                await self._call(adapter, 'org.bluez.Adapter1', 'StartDiscovery')
            except BluetoothError as e:
                if 'InProgress' not in str(e):
                    raise
            if self._discovery_task:
                self._discovery_task.cancel()
            self._discovery_task = asyncio.ensure_future(self._stop_discovery_later(adapter))
        else:
            if self._discovery_task:
                self._discovery_task.cancel()
                self._discovery_task = None
            try:
                await self._call(adapter, 'org.bluez.Adapter1', 'StopDiscovery')
            except BluetoothError:
                pass

    async def _stop_discovery_later(self, adapter):
        try:
            await asyncio.sleep(DISCOVERY_SECONDS)
            await self._call(adapter, 'org.bluez.Adapter1', 'StopDiscovery')
        except (asyncio.CancelledError, BluetoothError):
            pass

    async def action(self, address, action):
        if action not in ('pair', 'trust', 'untrust', 'connect', 'disconnect', 'remove'):
            raise BluetoothError('unknown action %r' % action)
        if self.busy.get(address):
            raise BluetoothError('%s already in progress for %s' % (self.busy[address], address))
        self.busy[address] = action
        try:
            adapter, path = await self._device_path(address)
            if action == 'pair':
                await self._ensure_agent()
                try:
                    await self._call(path, 'org.bluez.Device1', 'Pair', timeout=60.0)
                except BluetoothError as e:
                    if 'AlreadyExists' not in str(e):
                        raise
                # gamepads reconnect by themselves only when trusted
                await self._set_trusted(path, True)
            elif action == 'trust':
                await self._set_trusted(path, True)
            elif action == 'untrust':
                await self._set_trusted(path, False)
            elif action == 'connect':
                await self._call(path, 'org.bluez.Device1', 'Connect', timeout=30.0)
            elif action == 'disconnect':
                await self._call(path, 'org.bluez.Device1', 'Disconnect', timeout=15.0)
            elif action == 'remove':
                await self._call(adapter, 'org.bluez.Adapter1', 'RemoveDevice', 'o', [path])
        finally:
            self.busy.pop(address, None)

    async def _set_trusted(self, path, value):
        await self._call(path, 'org.freedesktop.DBus.Properties', 'Set', 'ssv',
                         ['org.bluez.Device1', 'Trusted', Variant('b', bool(value))])

    async def connected_controllers(self):
        try:
            return [d for d in await self.devices() if d['connected']]
        except BluetoothError:
            return []
