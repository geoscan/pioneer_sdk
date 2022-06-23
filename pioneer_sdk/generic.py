import logging
import pathlib
import os
import inspect
import time
import threading

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class GetattrLockDecorator:
	"""
	Creates a proxy which behaves exactly like the object it encapsulates, while locking
	it each time its attributes are get-accessed.

	Its primary goal is to store a locking primitive (mutex), and the protected instance itself
	"""

	class ObjLockGuard:
		"""
		Raii object locker
		"""

		def __init__(self, obj, lock):
			self.__lock = lock
			self.__lock.acquire()
			self.__obj = obj

		def __del__(self):
			self.__lock.release()

		def __getattr__(self, name):
			return self.__obj.__getattribute__(name)

	def __init__(self, obj):
		self.__lock = threading.Lock()
		self.__obj = obj

	def __getattr__(self, name):
		return getattr(GetattrLockDecorator.ObjLockGuard(self.__obj, self.__lock), name)


class Logging:

	@staticmethod
	def info(*args, **kwargs):
		return logging.info(Logging.format(*args, **kwargs))

	@staticmethod
	def warning(*args, **kwargs):
		return logging.warning(Logging.format(*args, **kwargs))

	@staticmethod
	def error(*args, **kwargs):
		return logging.error(Logging.format(*args, **kwargs))

	@staticmethod
	def debug(*args, **kwargs):
		return logging.debug(Logging.format(*args, **kwargs))

	@staticmethod
	def critical(*args, **kwargs):
		return logging.critical(Logging.format(*args, **kwargs))

	@staticmethod
	def format(*args, **kwargs):
		"""
		Formats input data according to the following pattern: "[CONTEXT] TOPICS (if any) | message".

		The context is inferred by detecting the following types of objects:
		- a string representing Path
		- type name
		- callable

		Topics get passed explicitly with `topics=LIST` argument
		"""

		context = []
		suffix = []

		def is_path(arg):
			if type(arg) is not str:
				return False
			return os.path.isfile(arg) or os.path.isdir(arg)

		def format_path(arg):
			return pathlib.Path(arg).stem

		def is_class(arg):
			return inspect.isclass(arg)

		def format_class(arg):
			return arg.__name__

		def format_callable(arg):
			return arg.__name__ + "()"

		for a in args:
			if is_path(a):
				context += [format_path(a)]
			elif is_class(a):
				context += [format_class(a)]
			elif callable(a):
				context += [format_callable(a)]
			else:
				suffix += [str(a)]

		topics = " "
		if "topics" in kwargs.keys():
			topics = kwargs["topics"]
			topics = ' ' + ', '.join(topics) + ' | '

		return '[' + ' : '.join(context) + ']' + topics + ' '.join(suffix)


START_TIME = time.time()


def uptime_sec():
	return int(time.time() - START_TIME)


def uptime_ms():
	return int((time.time() - START_TIME) * 1000)


def extend_bytes_zeros(b: bytes, required_length):
	b = b[0:required_length]
	n_append = required_length - len(b)

	return b + bytes([0]) * n_append


def try_unpack_fields(obj, *fields):
	if obj is None:
		return None
	else:
		return (getattr(obj, f) for f in fields)
