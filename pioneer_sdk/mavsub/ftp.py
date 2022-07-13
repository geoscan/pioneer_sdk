import struct
import sys
import pathlib
import re
import os

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from ..generic import Logging

TARGET_NETWORK = 0
RECV_TIMEOUT_SEC = 2


class Op:
    """ Standard MAVLink FTP subprotocol opcode values """

    @staticmethod
    def to_string(val):
        rev_dict = dict((v, k) for k, v in Op.__dict__.items() if type(v) is int)
        return rev_dict[val]

    NONE = 0
    TERMINATE_SESSION = 1
    RESET_SESSIONS = 2
    LIST_DIRECTORY = 3
    OPEN_FILE_RO = 4
    READ_FILE = 5
    CREATE_FILE = 6
    WRITE_FILE = 7
    REMOVE_FILE = 8
    CREATE_DIRECTORY = 9
    REMOVE_DIRECTORY = 10
    OPEN_FILE_WO = 11
    TRUNCATE_FILE = 12
    RENAME = 13
    CALC_FILE_CRC32 = 14
    BURST_READ_FILE = 15
    ACK = 128
    NAK = 129


# Error codes
class Nak(Exception):

    @staticmethod
    def to_string(nak):
        rev_dict = dict((v, k) for k, v in Nak.__dict__.items() if type(v) is int)
        try:
            return f'{rev_dict[nak]} ({nak})'
        except KeyError:
            return f"unknown error code ({nak})"

    @staticmethod
    def try_raise(nak):
        """
		:param nak: Standard MAVLink Nak code
		:return: If nak does signify a critical error, raises an appropriate exception
		"""
        if nak != Nak.NONE and nak != Nak.EOF:
            raise Nak(nak)

    def __init__(self, nak):
        Exception.__init__(self, Nak.to_string(nak))

    NONE = 0
    FAIL = 1
    FAIL_ERRNO = 2
    INVALID_DATA_SIZE = 3
    INVALID_SESSION = 4
    NO_SESSIONS_AVAILABLE = 5
    EOF = 6
    UNKNOWN_COMMAND = 7
    FILE_EXISTS = 8
    FILE_PROTECTED = 9
    FILE_NOT_FOUND = 10


class FtpPayload:
    OVERALL_LENGTH = 251
    PAYLOAD_LENGTH = 239
    HEADER_LENGTH = OVERALL_LENGTH - PAYLOAD_LENGTH

    def __init__(self, seq=0, session=0, opcode=0, size=0, req_opcode=0, burst_complete=0, offset=0, payload=b''):
        self.seq = seq
        self.session = session
        self.opcode = opcode
        self.size = size
        self.req_opcode = req_opcode
        self.burst_complete = burst_complete
        self.offset = offset
        self.payload = payload

    @property
    def nak(self):
        if self.opcode == Op.ACK:
            return Nak.NONE

        if len(self.payload) > 0:
            return int(self.payload[0])

        return Nak.FAIL

    @staticmethod
    def construct_from_bytes(ftp_payload):
        ftp_payload = bytearray(ftp_payload)

        if len(ftp_payload) < FtpPayload.HEADER_LENGTH:
            ftp_payload.extend(bytearray([0]) * (FtpPayload.HEADER_LENGTH - len(ftp_payload)))

        ret = struct.unpack("<HBBBBBxI", ftp_payload[0: FtpPayload.HEADER_LENGTH])

        # Extract ftp payload (data field)
        ftp_payload = ftp_payload[FtpPayload.HEADER_LENGTH:]
        ftp_payload += b'\x00'  # Apparently, "struct.unpack" cuts trailing zeros
        ftp_payload = ftp_payload[:ret[3]]  # Cut the tail.

        ret = ret + (ftp_payload,)
        ret = FtpPayload(*ret)

        if ret.size != len(ret.payload):
            Logging.error(__file__, FtpPayload, FtpPayload.construct_from_bytes,
                          "declared payload size and length of the payload don't match.", "Declared size:", ret.size,
                          "The actual payload's length:", len(ret.payload), topics=["serialization"])

        return ret

    def pack(self):
        """
		:return: bytearray representation of a message
		"""
        ret = struct.pack("<HBBBBBBI", self.seq, self.session, self.opcode, self.size, self.req_opcode,
                          self.burst_complete, 0, self.offset)
        if self.payload is not None:
            ret += bytearray(self.payload)
        remainder_length = FtpPayload.OVERALL_LENGTH - len(ret)
        ret += bytearray([0] * remainder_length)
        # ret = bytearray(ret)
        # print(len(ret))
        return ret

    def __str__(self):
        plen = 0

        if self.payload is not None:
            plen = len(self.payload)

        ret = "OP seq:%u sess:%u opcode:%s req_opcode:%s size:%u bc:%u ofs:%u plen=%u" % (self.seq,
                                                                                          self.session,
                                                                                          Op.to_string(self.opcode),
                                                                                          Op.to_string(self.req_opcode),
                                                                                          self.size,
                                                                                          self.burst_complete,
                                                                                          self.offset, plen)

        if plen > 0:
            ret += " [%s]" % self.payload[:]

        if self.opcode == Op.NAK and len(self.payload) > 0:
            ret += Nak.to_string(self.payload[0])

        return ret


_prev_args = None
_prev_kwargs = None
_prev_func = None


def increase_seq(func):
    def wrapper(self, *args, **kwargs):
        global _prev_args
        global _prev_kwargs
        global _prev_func

        # If we use the same function with the same parameters, chances are we making a second attempt to connect
        if not (_prev_args == args and _prev_kwargs == kwargs and _prev_func == func):
            self.seq = (self.seq + 1) % 65536

        ret = func(self, *args, **kwargs)
        _prev_func = func
        _prev_args = args
        _prev_kwargs = kwargs

        return ret

    return wrapper


class Ftp:
    """
	Wraps communication with the device over FTP sub-protocol.
	"""

    TYPE_LIST_RESPONSE_RE = r'(D|S|F)'
    NAME_LIST_RESPONSE_RE = r'([^\t\/]+)'
    SIZE_LIST_RESPONSE_RE = r'(-?[0-9]+)'
    LIST_RESPONSE_RE = TYPE_LIST_RESPONSE_RE + NAME_LIST_RESPONSE_RE + r'\t' + SIZE_LIST_RESPONSE_RE + r'\0'

    def __init__(self, connection, target_network=0):
        """
		:param connection: MAVLink connection instance
		:param sysid:  target component id
		:param compid: target system id
		:param target_network: target network (see description for MAVLink FTP subprotocol)
		"""

        self.connection = connection
        self.seq = 0
        self.target_network = target_network

    def send(self, payload):
        payload.seq = self.seq

        Logging.debug(__file__, Ftp, Ftp.send, "Sending payload: ", str(payload))

        self.connection.mav.file_transfer_protocol_send(self.target_network, self.connection.target_system,
                                                        self.connection.target_component, payload.pack())

    def receive(self):
        """
		:return: FtpPayload, or None
		"""
        # msg = self.connection.recv_match(type="FILE_TRANSFER_PROTOCOL",
        # 	condition=f"FILE_TRANSFER_PROTOCOL.seq=={self.seq}", blocking=True,
        # 	timeout=RECV_TIMEOUT_SEC)

        msg = self.connection.recv_match(type="FILE_TRANSFER_PROTOCOL", blocking=True, timeout=RECV_TIMEOUT_SEC)

        if not msg:
            Logging.warning(__file__, Ftp, Ftp.receive, "failed to receive", topics=['Conn'])
            return None

        return msg

    def receive_payload(self):
        """
		Decorator over self.receive that extracts payload
		"""
        msg = self.receive()

        if not msg:
            return None

        payload = FtpPayload.construct_from_bytes(msg.payload)

        if payload.seq != self.seq:
            Logging.info(__file__, Ftp, Ftp.receive_payload,
                         "Got payload, but `seq` fields don't match", "current seq:", self.seq, "received payload:",
                         str(payload), topics=["conn"])

            return None

        Logging.debug(__file__, Ftp, Ftp.receive_payload, "Received payload:", str(payload))

        return payload

    @increase_seq
    def list_directory(self, offset, file_path):
        """
		:param directory: Path to the requested directory, unix-like format
		:return: (NakCode, [("D"|"F"|"S", NAME, SIZE), ...]), or None if failed to get a response
		"""
        payload = FtpPayload(opcode=Op.LIST_DIRECTORY, size=len(file_path), offset=offset,
                             payload=bytearray(file_path, encoding='ascii'))
        self.send(payload)
        payload = self.receive_payload()

        # Message receiving has failed
        if not payload:
            return None

        # Payload is empty
        if payload.size == 0:
            return payload.nak, []

        # Unpack directory entries from payload

        ret = []

        for m in re.finditer(Ftp.LIST_RESPONSE_RE, payload.payload.decode('ascii')):
            type = m.group(1)
            name = m.group(2)
            size = int(m.group(3))
            ret += [(type, name, size,)]

        return payload.nak, ret

    @increase_seq
    def terminate_session(self, session):
        """
		:param session: Session ID
		:return: NakCode, or None if failed to get a response
		"""
        payload = FtpPayload(opcode=Op.TERMINATE_SESSION, session=session)
        self.send(payload)
        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def reset_sessions(self):
        """
		:return: NakCode, or None if failed to get a response
		"""
        payload = FtpPayload(opcode=Op.RESET_SESSIONS)
        self.send(payload)
        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def open_file_ro(self, file_path):
        """
		:param file_path: Path to the requested file, unix-like format
		:return: (NakCode, Session ID), or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.OPEN_FILE_RO, size=len(file_path),
                             payload=bytearray(file_path, encoding='ascii')))
        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak, payload.session

    @increase_seq
    def read_file(self, size, session, offset):
        """
		:param size: Size of the chunk to read
		:param session: Session ID associated with the requested file
		:param offset: Offset to read from
		:return: (NakCode, bytearray), or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.READ_FILE, size=size, session=session, offset=offset))

        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak, payload.payload

    @increase_seq
    def create_file(self, file_path):
        """
		:param file_path: Path to the requested file, unix-like format
		:return: (NakCode, Session ID), or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.READ_FILE, size=len(file_path), payload=bytearray(file_path, encoding='ascii')))
        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak, payload.session

    @increase_seq
    def write_file(self, session, offset, content):
        """

		:param session: Session ID associated with the requested file
		:param content: Payload to write to the file
		:return: NakCode, or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.WRITE_FILE, size=len(content), offset=offset, session=session,
                             payload=bytearray(content)))
        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def remove_file(self, file_path):
        """
		:param file_path: Path to the requested file, unix-like format
		:return: NakCode, or None if failed to get a response
		"""
        self.send(
            FtpPayload(opcode=Op.REMOVE_FILE, size=len(file_path), payload=bytearray(file_path, encoding='ascii')))

        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def create_directory(self, dir_path):
        """
		:param dir_path: Path to the requested file, unix-like format
		:return: NakCode, or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.CREATE_DIRECTORY, size=len(dir_path),
                             payload=bytearray(dir_path, encoding='ascii')))

        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def remove_directory(self, dir_path):
        """
		:param dir_path: Path to the requested directory, unix-like format
		:return: NakCode, or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.CREATE_DIRECTORY, size=len(dir_path),
                             payload=bytearray(dir_path, encoding='ascii')))

        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def open_file_wo(self, file_path):
        """
		:param file_path: Path to the requested file, unix-like format
		:return: (NakCode, Session ID), or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.OPEN_FILE_WO, size=len(file_path),
                             payload=bytearray(file_path, encoding='ascii')))

        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak, payload.session

    @increase_seq
    def truncate_file(self, offset, file_path):
        """
		:param offset: Offset for truncation operation
		:param file_path: Path to the requested file, unix-like format
		:return: NakCode, or None if failed to get a response
		"""
        self.send(FtpPayload(opcode=Op.TRUNCATE_FILE, size=len(file_path), offset=offset,
                             payload=bytearray(file_path, encoding='ascii')))

        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def rename(self, src_file_path, dest_file_path):
        """
		:param src_file_path: Path to the initial file, unix-like format
		:param dest_file_path: Path to the file after renaming, unix-like format
		:return: NakCode, or None if failed to get a response
		"""
        paths_packed = bytearray(src_file_path, encoding='ascii') + '\0' + bytearray(dest_file_path,
                                                                                     encoding='ascii') + '\0'

        self.send(FtpPayload(opcode=Op.RENAME, size=len(paths_packed), payload=paths_packed))
        payload = self.receive_payload()

        if not payload:
            return None

        return payload.nak

    @increase_seq
    def calc_file_crc32(self, file_path):
        """
		:param file_path: Path to the requested file, unix-like format
		:return: (NakCode, Crc32), or None if failed to get a response
		"""
        NETWORK_BYTE_ORDER = 'big'
        self.send(FtpPayload(opcode=Op.CALC_FILE_CRC32, size=len(file_path), payload=bytearray(file_path)))
        payload = self.receive_payload()  # TODO: payload calculation may take some time for larger files. Increase the timeout

        if not payload:
            return None

        crc32 = payload[:4]
        crc32 = int.from_bytes(crc32, byteorder=NETWORK_BYTE_ORDER, signed=False)

        return payload.nak, crc32


class FtpWrapper:
    """
	It handles the following aspects of communication:
	1. Connection monitoring
	2. Session management
	3. Request repetition
	"""

    # TODO: as for 2021-09-15, Geoscan MAVLink implementation kicks off file or directory creation/removal requests as those make no sense to the Autopilot. Implement relevant methods if and when it stops being the case

    N_RECEIVE_ATTEMPTS = 2
    CHUNK_SIZE = 100

    def __init__(self, mavlink_connection, target_network=0):
        self.ftp = Ftp(mavlink_connection, target_network)

    @staticmethod
    def _try_receive(callable, *args, **kwargs):
        res = None

        for _ in range(FtpWrapper.N_RECEIVE_ATTEMPTS):
            res = callable(*args, **kwargs)

            if res is not None:
                return res

        raise ConnectionError(f"Couldn't get a response in {FtpWrapper.N_RECEIVE_ATTEMPTS} attempts")

    def download_file(self, src, dest) -> bytes or None:
        """
		:param src: servers-side path
		:param dest: client-side path, or None, if the result should be stored in RAM
		:return: bytes if dest is None. None otherwise
		"""

        class ReadState:
            """
			Manages offset shifting and file I/O, if necessary (which depends on whether `dest == None`)
			"""

            def __init__(self):
                self._file = open(dest, "wb") if dest is not None else None
                self._chunk = b''
                self.offset = 0

            @property
            def result(self):
                if dest is None:
                    return None

                return self._chunk

            def __del__(self):
                if self._file is not None:
                    self._file.close()

            def handle_chunk(self, chunk):
                # Write into the file if dest is not None. Accumulate result otherwise
                if dest is not None:
                    self._file.write(chunk)
                else:
                    self._chunk += chunk

                self.offset += len(chunk)

        # Create session
        nak, sid = self._try_receive(self.ftp.open_file_ro, src)
        Nak.try_raise(nak)

        # Try to read the file

        nak = Nak.NONE
        read_state = ReadState()

        while nak == Nak.NONE:
            nak, chunk = self._try_receive(self.ftp.read_file, size=FtpWrapper.CHUNK_SIZE, session=sid,
                                           offset=read_state.offset)
            Nak.try_raise(nak)
            if nak == Nak.NONE:
                read_state.handle_chunk(chunk)

        # Close session
        nak = self._try_receive(self.ftp.terminate_session, sid)
        Nak.try_raise(nak)

        return read_state.result

    def upload_file(self, src, dest) -> type(Nak.NONE):
        """
		:param src: client-side path of type `str`, or bytes
		:param dest: server-side path
		:return: None, on success. Exception otherwise
		"""

        class WriteState:
            def __init__(self):
                assert type(src) in [str, bytes, bytearray]

                self._offset = 0
                src_path = type(src) is str and os.path.isfile(src)

                self._file = open(src, 'rb') if src_path else None

            def __del__(self):
                if self._file is not None:
                    self._file.close()

            def handle_chunk(self):
                if self._file is None:
                    next_chunk = src[self._offset: self._offset + FtpWrapper.CHUNK_SIZE]
                else:
                    next_chunk = self._file.read(FtpWrapper.CHUNK_SIZE)

                off = self._offset
                self._offset += len(next_chunk)

                return off, next_chunk

        # Get a session id

        nak, sid = self._try_receive(self.ftp.open_file_wo, dest)
        Nak.try_raise(nak)

        # Write a file chunk by chunk

        write_state = WriteState()
        offset, chunk = write_state.handle_chunk()

        while len(chunk):
            nak = self._try_receive(self.ftp.write_file, sid, offset, chunk)
            Nak.try_raise(nak)

            offset, chunk = write_state.handle_chunk()

        # Terminate the session

        nak = self._try_receive(self.ftp.terminate_session, sid)
        Nak.try_raise(nak)

    def list_directory(self, path) -> None:
        """
		:param path: server-side path
		:return: List of directories, on successful receive. Exception otherwise
		"""
        nak = Nak.NONE
        file_list = []
        read_offset = 0

        while nak == Nak.NONE:
            nak, fl = self._try_receive(self.ftp.list_directory, offset=read_offset, file_path=path)
            file_list += fl
            Nak.try_raise(nak)
            read_offset += len(fl)

        return file_list

    def reset_sessions(self):
        nak = self._try_receive(self.ftp.reset_sessions)
