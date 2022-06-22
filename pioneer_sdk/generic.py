import logging
import pathlib
import os
import inspect
import time


class Logging:

	LEVEL = logging.DEBUG
	_logger = None

	@staticmethod
	def get_logger():

		if Logging._logger is None:
			Logging._logger = logging.getLogger('logger')
			Logging._logger.setLevel(Logging.LEVEL)

			sh = logging.StreamHandler()
			sh.setLevel(Logging.LEVEL)
			sh.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

			Logging._logger.addHandler(sh)

		return Logging._logger

	@staticmethod
	def info(*args, **kwargs):
		return Logging.get_logger().info(Logging.format(*args, **kwargs))

	@staticmethod
	def warning(*args, **kwargs):
		return Logging.get_logger().warning(Logging.format(*args, **kwargs))

	@staticmethod
	def error(*args, **kwargs):
		return Logging.get_logger().error(Logging.format(*args, **kwargs))

	@staticmethod
	def debug(*args, **kwargs):
		return Logging.get_logger().debug(Logging.format(*args, **kwargs))

	@staticmethod
	def critical(*args, **kwargs):
		return Logging.get_logger().critical(Logging.format(*args, **kwargs))

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
