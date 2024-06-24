import socket
import urllib.parse
import orjson
import msgpack
import torch
import numpy as np
import select
import threading


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
        encoding: str = 'msgpack',
        broadcast: bool = False,
        enable: bool = True,
        **socket_kwargs,
    ):
        """
        Initialize DataPublisher with connection and encoding details.

        Parameters
        ----------
        target_url : str
            URL to which data will be sent.
        encoding : str
            Encoding for sending data, default is 'json'.
        broadcast : bool
            If True, data is broadcasted; defaults to True.
        enable : bool
            If False, publishing is inactive; defaults to False.
        socket_kwargs : 
            Additional keyword arguments for the socket.
        """
        self.enable = enable
        self.url = urllib.parse.urlparse(target_url)

        # Validate scheme
        if self.url.scheme not in self.SUPPORTED_SCHEMES:
            raise ValueError(f"Unsupported scheme in URL: {target_url}")

        # Set socket family and type
        family = self._get_socket_family()
        socket_type = socket.SOCK_DGRAM if 'udp' in self.url.scheme else socket.SOCK_STREAM

        # Create socket
        self.socket = socket.socket(family, socket_type, **socket_kwargs)
        if broadcast:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.hostname = '<broadcast>' if broadcast else self.url.hostname

        # Set encoding function
        self._setup_encoding(encoding)

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
        if self.enable:
            converted_data = convert_to_python_builtin_types(data)
            encoded_data = self.encode(converted_data)
            self.socket.sendto(encoded_data, (self.hostname, self.url.port))


class DataReceiver:
    """
    Receives data published by a DataPublisher instance using a non-blocking socket.
    """

    def __init__(
            self, target_port: int = 9870, decoding: str = "msgpack"):
        """
        Initializes the DataReceiver.

        Args:
            target_port (int): The port to listen on for incoming data. Defaults to 9870.
            decoding (str): The decoding method for data (raw/utf-8/msgpack/json). Defaults to "msgpack".
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
        if decoding not in decodings:
            raise ValueError(f"Invalid decoding: {decoding}")
        self.decode = decodings[decoding]
        self.data = None
        self.address = None
        self._running = True  # Flag to control the receive loop

    def receive(self, timeout=0.1, buffer_size=1024):
        """Receive and decode data from the socket if available, otherwise return None."""
        ready = select.select([self.socket], [], [], timeout)
        if ready[0]:
            data, self.address = self.socket.recvfrom(buffer_size)
            self.data = self.decode(data)
            # print(f"Received from {self.address}: {self.data}")
            return self.data, self.address  # Return decoded data and sender address
        else:
            return None, None  # No data received within the timeout

    def receive_continuously(self, timeout=0.1, buffer_size=1024):
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


if __name__ == "__main__":
    import time

    # Sample data to publish
    time_since_start = time.time()

    def test_data():
        """example test data"""
        return {
            "time": time.time()-time_since_start,
            "sensor_id": np.random.randint(0, 10),
            "temperature": 25.5,
            "humidity": 68
        }

    # Create a publisher instance
    publisher = DataPublisher(
        target_url="udp://localhost:9871", encoding="msgpack")

    # Create a receiver instance
    receiver = DataReceiver(target_port=9871, decoding="msgpack")

    # Start continuous receiving in a thread
    receiver.receive_continuously()

    # Send data multiple times with a delay
    for i in range(10):
        publisher.publish(test_data())
        print(f"Received from {receiver.address}: {receiver.data}")
        time.sleep(0.001)  # add a small delay

    # Stop continuous receiving after a while
    receiver.stop()

    print("Publisher and Receiver have stopped.")
