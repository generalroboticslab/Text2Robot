import socket
import urllib.parse
import orjson
import msgpack
import torch
import numpy as np
import select
import threading
import subprocess
from pylx16a.lx16a import *
import time
import numpy as np
LX16A.initialize("/dev/ttyUSB0", 0.1) # Change port based on which port is connected to the servo controller

try:
    servo0 = LX16A(0)  # Ensure that real servos match isaacgym indecies, or realign here
    servo1 = LX16A(4)
    servo2 = LX16A(1)
    servo3 = LX16A(5)
    servo4 = LX16A(2)
    servo5 = LX16A(6)
    servo6 = LX16A(3)
    servo7 = LX16A(7)
    servo0.set_angle_limits(0, 240)  # Set safe limits for servos
    servo1.set_angle_limits(0, 240)
    servo2.set_angle_limits(0, 240)
    servo3.set_angle_limits(0, 240)
    servo4.set_angle_limits(0, 240)
    servo5.set_angle_limits(0, 240)
    servo6.set_angle_limits(0, 240)
    servo7.set_angle_limits(0, 240)

except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

class DataPublisher:
    """
    Manages data publishing to a specified target URL.

    Methods
    -------
    publish(data: dict)
        Sends data to the target URL if publishing is enabled.
    """

    SUPPORTED_SCHEMES = {'unix', 'tcp', 'udp'}

    def __init__(
        self,
        target_url: str = 'udp://localhost:9870',
        encoding_type: str = 'msgpack',
        is_broadcast: bool = False,
        is_enabled: bool = True,
        **socket_kwargs,
    ):
        """
        Initialize DataPublisher with connection and encoding details.

        Parameters
        ----------
        target_url : str
            URL to which data will be sent.
        encoding_type : str
            Encoding for sending data, default is 'json'.
        is_broadcast : bool
            If True, data is broadcasted; defaults to True.
        is_enabled : bool
            If False, publishing is inactive; defaults to False.
        socket_kwargs : 
            Additional keyword arguments for the socket.
        """
        self.is_enabled = is_enabled
        self.url = urllib.parse.urlparse(target_url)

        # Validate scheme
        if self.url.scheme not in self.SUPPORTED_SCHEMES:
            raise ValueError(f"Unsupported scheme in URL: {target_url}")

        # Set socket family and type
        family = self._get_socket_family()
        socket_type = socket.SOCK_DGRAM if 'udp' in self.url.scheme else socket.SOCK_STREAM

        # Create socket
        self.socket = socket.socket(family, socket_type, **socket_kwargs)
        if is_broadcast:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.hostname = '<broadcast>' if is_broadcast else self.url.hostname

        # Set encoding function
        self._setup_encoding(encoding_type)

    def _get_socket_family(self):
        """Determine and return the appropriate socket family based on the URL."""
        if 'unix' in self.url.scheme:
            return socket.AF_UNIX
        return socket.AF_INET6 if ':' in (self.url.hostname or '') else socket.AF_INET

    def _setup_encoding(self, encoding_type):
        """Configure the data encoding method based on the provided encoding_type."""
        encodings = {
            'raw': lambda data: data,  # raw/bytes
            'utf-8': lambda data: data.encode('utf-8'),
            'msgpack': lambda data: msgpack.packb(data, use_single_float=False, use_bin_type=True),
            'json': lambda data: orjson.dumps(data),
        }
        if encoding_type not in encodings:
            raise ValueError(f'Invalid encoding: {encoding_type}')
        self.encode = encodings[encoding_type]

    def publish(self, data: dict):
        """
        Publishes the provided data to the target URL.

        Parameters
        ----------
        data : dict
            Data to be published.
        """
        if self.is_enabled:
            converted_data = convert_to_python_builtin_types(data)
            encoded_data = self.encode(converted_data)
            self.socket.sendto(encoded_data, (self.hostname, self.url.port))


class DataReceiver:
    """
    Receives data published by a DataPublisher instance using a non-blocking socket.
    """

    def __init__(
        self, target_port: int = 9871, decoding_type: str = "msgpack"):
        """
        Initializes the DataReceiver.

        Args:
            target_port (int): The port to listen on for incoming data. Defaults to 9870.
            decoding_type (str): The decoding method for data ("msgpack" or "json"). Defaults to "msgpack".
        """   
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("", target_port))
        self.socket.setblocking(False)
        decodings = {
            "raw": lambda data: data,  # raw/bytes
            "utf-8": lambda data: data.decode("utf-8"),
            "msgpack": lambda data: msgpack.unpackb(data, raw=False),
            "json": lambda data: orjson.loads(data),
        }
        if decoding_type not in decodings:
            raise ValueError(f"Invalid decoding: {decoding_type}")
        self.decode = decodings[decoding_type]
        self.data = None
        self.address = None
        self._running = True  # Flag to control the receive loop
        self.get_server_local_ip()
        self.get_server_public_ip()
        input('input to start')

    def receive(self, timeout=0.1, buffer_size=2048):
        """Receive and decode data from the socket if available, otherwise return None."""
        ready = select.select([self.socket], [], [], timeout)
        if ready[0]:
            data, self.address = self.socket.recvfrom(buffer_size)
            self.data = self.decode(data)
            # print(f"Received from {self.address}: {self.data}")
            return self.data, self.address  # Return decoded data and sender address
        else:
            return None, None  # No data received within the timeout

    def receive_continuously(self,timeout=0.1, buffer_size=2048):
        """Continuously receive and decode data from the socket in a dedicated thread."""

        def _receive_loop():
            while self._running:
                self.receive(timeout=timeout, buffer_size=buffer_size)

        thread = threading.Thread(target=_receive_loop, daemon=True)
        thread.start()

    def stop(self):
        """Stop continuous receiving."""
        self._running = False
        self.socket.close()
    def get_server_local_ip(verbose=True):
        local_ip = socket.gethostbyname(socket.gethostname())
        if verbose:
            print(f"#################"
                f"Server Local IP Address: {local_ip}"
                f"#################")
        return local_ip


    @staticmethod
    def get_server_public_ip(verbose=True):
        # Execute the hostname -I command and capture its output
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)

        # Split the output by whitespace to get individual IP addresses
        ip_addresses = result.stdout.strip().split()
        
        # Check if there is a second IP address available
        if len(ip_addresses) > 0:
            if verbose:
                print(f"#################"
                    f"Server Public IP Address: {ip_addresses[-1]}"
                    f"#################")
            return ip_addresses[-1]  # Second IP address
        else:
            raise Exception("No Server Public IP address found; Please check your internet connection")

def convert_to_python_builtin_types(nested_data: dict):
    """
    Converts nested data (including tensors and arrays) to built-in types.

    Parameters
    ----------
    nested_data : dict
        Data to be converted.

    Returns
    -------
    dict
        Data converted to Python built-in types.
    """
    converted_data = {}
    for key, value in nested_data.items():
        if isinstance(value, dict):
            converted_data[key] = convert_to_python_builtin_types(value)
        elif isinstance(value, (torch.Tensor, np.ndarray)):
            converted_data[key] = value.tolist()
        else:
            converted_data[key] = value
    return converted_data

if __name__=="__main__":
    import time

    # Sample data to publish
    time_since_start = time.time()
    def test_data():
        """example test data"""
        return {"time":time.time()-time_since_start,"sensor_id": np.random.randint(0,10), "temperature": 25.5, "humidity": 68}

    # Create a publisher instance
    # publisher = DataPublisher(target_url="udp://localhost:9871", encoding_type="msgpack")

    # Create a receiver instance
    receiver = DataReceiver(target_port=9871, decoding_type="msgpack")

    # Start continuous receiving in a thread
    receiver.receive_continuously()
    init_speed = 200
    speed = 15
    i = 0

    # Gain should be adjusted to match simulation performance
    shoulder_gain = 1.5

    # Home the shoulders
    servo0.move(120,init_speed)
    servo1.move(120,init_speed)
    servo2.move(120, init_speed)
    servo3.move(120,init_speed)
    servo4.move(120,init_speed)
    servo5.move(120,init_speed)
    servo6.move(120,init_speed)
    servo7.move(120,init_speed)

    input()

    while True:
        # print(f"Received from {receiver.address}: {receiver.data}")

        if i == 0:
            time.sleep(1)
            i = i + 1

        angle = receiver.data['dof_pos']
        print(receiver.data)
        if angle is not None:
            
            angle_0 = shoulder_gain * np.degrees(angle[0]) + 120 # pos
            angle_1 = np.degrees(angle[1]) + 120 # pos
            angle_2 = shoulder_gain * np.degrees(angle[2]) + 120 # pos
            angle_3 = np.degrees(-1 * angle[3]) + 120
            angle_4 = shoulder_gain * np.degrees(-1 * angle[4]) + 120
            angle_5 = np.degrees(angle[5]) + 120 # pos
            angle_6 = shoulder_gain * np.degrees(-1 * angle[6]) + 120
            angle_7 = np.degrees(-1 * angle[7]) + 120

            servo0.move(angle_0, speed)
            servo1.move(angle_1, speed)
            servo2.move(angle_2, speed)
            servo3.move(angle_3, speed)
            servo4.move(angle_4, speed)
            servo5.move(angle_5, speed)
            servo6.move(angle_6, speed)
            servo7.move(angle_7, speed)
