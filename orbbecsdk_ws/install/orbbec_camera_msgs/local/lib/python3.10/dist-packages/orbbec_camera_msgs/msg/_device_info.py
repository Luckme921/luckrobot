# generated from rosidl_generator_py/resource/_idl.py.em
# with input from orbbec_camera_msgs:msg/DeviceInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DeviceInfo(type):
    """Metaclass of message 'DeviceInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('orbbec_camera_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'orbbec_camera_msgs.msg.DeviceInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__device_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__device_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__device_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__device_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__device_info

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DeviceInfo(metaclass=Metaclass_DeviceInfo):
    """Message class 'DeviceInfo'."""

    __slots__ = [
        '_header',
        '_name',
        '_serial_number',
        '_firmware_version',
        '_supported_min_sdk_version',
        '_current_sdk_version',
        '_hardware_version',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'string',
        'serial_number': 'string',
        'firmware_version': 'string',
        'supported_min_sdk_version': 'string',
        'current_sdk_version': 'string',
        'hardware_version': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', str())
        self.serial_number = kwargs.get('serial_number', str())
        self.firmware_version = kwargs.get('firmware_version', str())
        self.supported_min_sdk_version = kwargs.get('supported_min_sdk_version', str())
        self.current_sdk_version = kwargs.get('current_sdk_version', str())
        self.hardware_version = kwargs.get('hardware_version', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.name != other.name:
            return False
        if self.serial_number != other.serial_number:
            return False
        if self.firmware_version != other.firmware_version:
            return False
        if self.supported_min_sdk_version != other.supported_min_sdk_version:
            return False
        if self.current_sdk_version != other.current_sdk_version:
            return False
        if self.hardware_version != other.hardware_version:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def serial_number(self):
        """Message field 'serial_number'."""
        return self._serial_number

    @serial_number.setter
    def serial_number(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'serial_number' field must be of type 'str'"
        self._serial_number = value

    @builtins.property
    def firmware_version(self):
        """Message field 'firmware_version'."""
        return self._firmware_version

    @firmware_version.setter
    def firmware_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'firmware_version' field must be of type 'str'"
        self._firmware_version = value

    @builtins.property
    def supported_min_sdk_version(self):
        """Message field 'supported_min_sdk_version'."""
        return self._supported_min_sdk_version

    @supported_min_sdk_version.setter
    def supported_min_sdk_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'supported_min_sdk_version' field must be of type 'str'"
        self._supported_min_sdk_version = value

    @builtins.property
    def current_sdk_version(self):
        """Message field 'current_sdk_version'."""
        return self._current_sdk_version

    @current_sdk_version.setter
    def current_sdk_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'current_sdk_version' field must be of type 'str'"
        self._current_sdk_version = value

    @builtins.property
    def hardware_version(self):
        """Message field 'hardware_version'."""
        return self._hardware_version

    @hardware_version.setter
    def hardware_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'hardware_version' field must be of type 'str'"
        self._hardware_version = value
