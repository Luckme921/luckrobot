# generated from rosidl_generator_py/resource/_idl.py.em
# with input from orbbec_camera_msgs:msg/DeviceStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DeviceStatus(type):
    """Metaclass of message 'DeviceStatus'."""

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
                'orbbec_camera_msgs.msg.DeviceStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__device_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__device_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__device_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__device_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__device_status

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


class DeviceStatus(metaclass=Metaclass_DeviceStatus):
    """Message class 'DeviceStatus'."""

    __slots__ = [
        '_header',
        '_color_frame_rate_cur',
        '_color_frame_rate_avg',
        '_color_frame_rate_min',
        '_color_frame_rate_max',
        '_color_delay_ms_cur',
        '_color_delay_ms_avg',
        '_color_delay_ms_min',
        '_color_delay_ms_max',
        '_depth_frame_rate_cur',
        '_depth_frame_rate_avg',
        '_depth_frame_rate_min',
        '_depth_frame_rate_max',
        '_depth_delay_ms_cur',
        '_depth_delay_ms_avg',
        '_depth_delay_ms_min',
        '_depth_delay_ms_max',
        '_device_online',
        '_connection_type',
        '_customer_calibration_ready',
        '_calibration_from_factory',
        '_calibration_from_launch_param',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'color_frame_rate_cur': 'double',
        'color_frame_rate_avg': 'double',
        'color_frame_rate_min': 'double',
        'color_frame_rate_max': 'double',
        'color_delay_ms_cur': 'double',
        'color_delay_ms_avg': 'double',
        'color_delay_ms_min': 'double',
        'color_delay_ms_max': 'double',
        'depth_frame_rate_cur': 'double',
        'depth_frame_rate_avg': 'double',
        'depth_frame_rate_min': 'double',
        'depth_frame_rate_max': 'double',
        'depth_delay_ms_cur': 'double',
        'depth_delay_ms_avg': 'double',
        'depth_delay_ms_min': 'double',
        'depth_delay_ms_max': 'double',
        'device_online': 'boolean',
        'connection_type': 'string',
        'customer_calibration_ready': 'boolean',
        'calibration_from_factory': 'boolean',
        'calibration_from_launch_param': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.color_frame_rate_cur = kwargs.get('color_frame_rate_cur', float())
        self.color_frame_rate_avg = kwargs.get('color_frame_rate_avg', float())
        self.color_frame_rate_min = kwargs.get('color_frame_rate_min', float())
        self.color_frame_rate_max = kwargs.get('color_frame_rate_max', float())
        self.color_delay_ms_cur = kwargs.get('color_delay_ms_cur', float())
        self.color_delay_ms_avg = kwargs.get('color_delay_ms_avg', float())
        self.color_delay_ms_min = kwargs.get('color_delay_ms_min', float())
        self.color_delay_ms_max = kwargs.get('color_delay_ms_max', float())
        self.depth_frame_rate_cur = kwargs.get('depth_frame_rate_cur', float())
        self.depth_frame_rate_avg = kwargs.get('depth_frame_rate_avg', float())
        self.depth_frame_rate_min = kwargs.get('depth_frame_rate_min', float())
        self.depth_frame_rate_max = kwargs.get('depth_frame_rate_max', float())
        self.depth_delay_ms_cur = kwargs.get('depth_delay_ms_cur', float())
        self.depth_delay_ms_avg = kwargs.get('depth_delay_ms_avg', float())
        self.depth_delay_ms_min = kwargs.get('depth_delay_ms_min', float())
        self.depth_delay_ms_max = kwargs.get('depth_delay_ms_max', float())
        self.device_online = kwargs.get('device_online', bool())
        self.connection_type = kwargs.get('connection_type', str())
        self.customer_calibration_ready = kwargs.get('customer_calibration_ready', bool())
        self.calibration_from_factory = kwargs.get('calibration_from_factory', bool())
        self.calibration_from_launch_param = kwargs.get('calibration_from_launch_param', bool())

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
        if self.color_frame_rate_cur != other.color_frame_rate_cur:
            return False
        if self.color_frame_rate_avg != other.color_frame_rate_avg:
            return False
        if self.color_frame_rate_min != other.color_frame_rate_min:
            return False
        if self.color_frame_rate_max != other.color_frame_rate_max:
            return False
        if self.color_delay_ms_cur != other.color_delay_ms_cur:
            return False
        if self.color_delay_ms_avg != other.color_delay_ms_avg:
            return False
        if self.color_delay_ms_min != other.color_delay_ms_min:
            return False
        if self.color_delay_ms_max != other.color_delay_ms_max:
            return False
        if self.depth_frame_rate_cur != other.depth_frame_rate_cur:
            return False
        if self.depth_frame_rate_avg != other.depth_frame_rate_avg:
            return False
        if self.depth_frame_rate_min != other.depth_frame_rate_min:
            return False
        if self.depth_frame_rate_max != other.depth_frame_rate_max:
            return False
        if self.depth_delay_ms_cur != other.depth_delay_ms_cur:
            return False
        if self.depth_delay_ms_avg != other.depth_delay_ms_avg:
            return False
        if self.depth_delay_ms_min != other.depth_delay_ms_min:
            return False
        if self.depth_delay_ms_max != other.depth_delay_ms_max:
            return False
        if self.device_online != other.device_online:
            return False
        if self.connection_type != other.connection_type:
            return False
        if self.customer_calibration_ready != other.customer_calibration_ready:
            return False
        if self.calibration_from_factory != other.calibration_from_factory:
            return False
        if self.calibration_from_launch_param != other.calibration_from_launch_param:
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
    def color_frame_rate_cur(self):
        """Message field 'color_frame_rate_cur'."""
        return self._color_frame_rate_cur

    @color_frame_rate_cur.setter
    def color_frame_rate_cur(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_frame_rate_cur' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_frame_rate_cur' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_frame_rate_cur = value

    @builtins.property
    def color_frame_rate_avg(self):
        """Message field 'color_frame_rate_avg'."""
        return self._color_frame_rate_avg

    @color_frame_rate_avg.setter
    def color_frame_rate_avg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_frame_rate_avg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_frame_rate_avg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_frame_rate_avg = value

    @builtins.property
    def color_frame_rate_min(self):
        """Message field 'color_frame_rate_min'."""
        return self._color_frame_rate_min

    @color_frame_rate_min.setter
    def color_frame_rate_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_frame_rate_min' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_frame_rate_min' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_frame_rate_min = value

    @builtins.property
    def color_frame_rate_max(self):
        """Message field 'color_frame_rate_max'."""
        return self._color_frame_rate_max

    @color_frame_rate_max.setter
    def color_frame_rate_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_frame_rate_max' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_frame_rate_max' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_frame_rate_max = value

    @builtins.property
    def color_delay_ms_cur(self):
        """Message field 'color_delay_ms_cur'."""
        return self._color_delay_ms_cur

    @color_delay_ms_cur.setter
    def color_delay_ms_cur(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_delay_ms_cur' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_delay_ms_cur' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_delay_ms_cur = value

    @builtins.property
    def color_delay_ms_avg(self):
        """Message field 'color_delay_ms_avg'."""
        return self._color_delay_ms_avg

    @color_delay_ms_avg.setter
    def color_delay_ms_avg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_delay_ms_avg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_delay_ms_avg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_delay_ms_avg = value

    @builtins.property
    def color_delay_ms_min(self):
        """Message field 'color_delay_ms_min'."""
        return self._color_delay_ms_min

    @color_delay_ms_min.setter
    def color_delay_ms_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_delay_ms_min' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_delay_ms_min' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_delay_ms_min = value

    @builtins.property
    def color_delay_ms_max(self):
        """Message field 'color_delay_ms_max'."""
        return self._color_delay_ms_max

    @color_delay_ms_max.setter
    def color_delay_ms_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'color_delay_ms_max' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'color_delay_ms_max' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._color_delay_ms_max = value

    @builtins.property
    def depth_frame_rate_cur(self):
        """Message field 'depth_frame_rate_cur'."""
        return self._depth_frame_rate_cur

    @depth_frame_rate_cur.setter
    def depth_frame_rate_cur(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_frame_rate_cur' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_frame_rate_cur' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_frame_rate_cur = value

    @builtins.property
    def depth_frame_rate_avg(self):
        """Message field 'depth_frame_rate_avg'."""
        return self._depth_frame_rate_avg

    @depth_frame_rate_avg.setter
    def depth_frame_rate_avg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_frame_rate_avg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_frame_rate_avg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_frame_rate_avg = value

    @builtins.property
    def depth_frame_rate_min(self):
        """Message field 'depth_frame_rate_min'."""
        return self._depth_frame_rate_min

    @depth_frame_rate_min.setter
    def depth_frame_rate_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_frame_rate_min' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_frame_rate_min' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_frame_rate_min = value

    @builtins.property
    def depth_frame_rate_max(self):
        """Message field 'depth_frame_rate_max'."""
        return self._depth_frame_rate_max

    @depth_frame_rate_max.setter
    def depth_frame_rate_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_frame_rate_max' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_frame_rate_max' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_frame_rate_max = value

    @builtins.property
    def depth_delay_ms_cur(self):
        """Message field 'depth_delay_ms_cur'."""
        return self._depth_delay_ms_cur

    @depth_delay_ms_cur.setter
    def depth_delay_ms_cur(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_delay_ms_cur' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_delay_ms_cur' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_delay_ms_cur = value

    @builtins.property
    def depth_delay_ms_avg(self):
        """Message field 'depth_delay_ms_avg'."""
        return self._depth_delay_ms_avg

    @depth_delay_ms_avg.setter
    def depth_delay_ms_avg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_delay_ms_avg' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_delay_ms_avg' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_delay_ms_avg = value

    @builtins.property
    def depth_delay_ms_min(self):
        """Message field 'depth_delay_ms_min'."""
        return self._depth_delay_ms_min

    @depth_delay_ms_min.setter
    def depth_delay_ms_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_delay_ms_min' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_delay_ms_min' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_delay_ms_min = value

    @builtins.property
    def depth_delay_ms_max(self):
        """Message field 'depth_delay_ms_max'."""
        return self._depth_delay_ms_max

    @depth_delay_ms_max.setter
    def depth_delay_ms_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_delay_ms_max' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'depth_delay_ms_max' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._depth_delay_ms_max = value

    @builtins.property
    def device_online(self):
        """Message field 'device_online'."""
        return self._device_online

    @device_online.setter
    def device_online(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'device_online' field must be of type 'bool'"
        self._device_online = value

    @builtins.property
    def connection_type(self):
        """Message field 'connection_type'."""
        return self._connection_type

    @connection_type.setter
    def connection_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'connection_type' field must be of type 'str'"
        self._connection_type = value

    @builtins.property
    def customer_calibration_ready(self):
        """Message field 'customer_calibration_ready'."""
        return self._customer_calibration_ready

    @customer_calibration_ready.setter
    def customer_calibration_ready(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'customer_calibration_ready' field must be of type 'bool'"
        self._customer_calibration_ready = value

    @builtins.property
    def calibration_from_factory(self):
        """Message field 'calibration_from_factory'."""
        return self._calibration_from_factory

    @calibration_from_factory.setter
    def calibration_from_factory(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'calibration_from_factory' field must be of type 'bool'"
        self._calibration_from_factory = value

    @builtins.property
    def calibration_from_launch_param(self):
        """Message field 'calibration_from_launch_param'."""
        return self._calibration_from_launch_param

    @calibration_from_launch_param.setter
    def calibration_from_launch_param(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'calibration_from_launch_param' field must be of type 'bool'"
        self._calibration_from_launch_param = value
