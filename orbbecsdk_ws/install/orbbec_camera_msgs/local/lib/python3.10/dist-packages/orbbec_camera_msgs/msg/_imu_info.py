# generated from rosidl_generator_py/resource/_idl.py.em
# with input from orbbec_camera_msgs:msg/IMUInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'bias'
# Member 'gravity'
# Member 'scale_misalignment'
# Member 'temperature_slope'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IMUInfo(type):
    """Metaclass of message 'IMUInfo'."""

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
                'orbbec_camera_msgs.msg.IMUInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imu_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imu_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imu_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imu_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imu_info

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


class IMUInfo(metaclass=Metaclass_IMUInfo):
    """Message class 'IMUInfo'."""

    __slots__ = [
        '_header',
        '_noise_density',
        '_random_walk',
        '_reference_temperature',
        '_bias',
        '_gravity',
        '_scale_misalignment',
        '_temperature_slope',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'noise_density': 'double',
        'random_walk': 'double',
        'reference_temperature': 'double',
        'bias': 'double[3]',
        'gravity': 'double[3]',
        'scale_misalignment': 'double[9]',
        'temperature_slope': 'double[9]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.noise_density = kwargs.get('noise_density', float())
        self.random_walk = kwargs.get('random_walk', float())
        self.reference_temperature = kwargs.get('reference_temperature', float())
        if 'bias' not in kwargs:
            self.bias = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.bias = kwargs.get('bias')
        if 'gravity' not in kwargs:
            self.gravity = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.gravity = kwargs.get('gravity')
        if 'scale_misalignment' not in kwargs:
            self.scale_misalignment = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.scale_misalignment = kwargs.get('scale_misalignment')
        if 'temperature_slope' not in kwargs:
            self.temperature_slope = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.temperature_slope = kwargs.get('temperature_slope')

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
        if self.noise_density != other.noise_density:
            return False
        if self.random_walk != other.random_walk:
            return False
        if self.reference_temperature != other.reference_temperature:
            return False
        if any(self.bias != other.bias):
            return False
        if any(self.gravity != other.gravity):
            return False
        if any(self.scale_misalignment != other.scale_misalignment):
            return False
        if any(self.temperature_slope != other.temperature_slope):
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
    def noise_density(self):
        """Message field 'noise_density'."""
        return self._noise_density

    @noise_density.setter
    def noise_density(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'noise_density' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'noise_density' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._noise_density = value

    @builtins.property
    def random_walk(self):
        """Message field 'random_walk'."""
        return self._random_walk

    @random_walk.setter
    def random_walk(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'random_walk' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'random_walk' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._random_walk = value

    @builtins.property
    def reference_temperature(self):
        """Message field 'reference_temperature'."""
        return self._reference_temperature

    @reference_temperature.setter
    def reference_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'reference_temperature' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'reference_temperature' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._reference_temperature = value

    @builtins.property
    def bias(self):
        """Message field 'bias'."""
        return self._bias

    @bias.setter
    def bias(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'bias' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'bias' numpy.ndarray() must have a size of 3"
            self._bias = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'bias' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._bias = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def gravity(self):
        """Message field 'gravity'."""
        return self._gravity

    @gravity.setter
    def gravity(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'gravity' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'gravity' numpy.ndarray() must have a size of 3"
            self._gravity = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'gravity' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._gravity = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def scale_misalignment(self):
        """Message field 'scale_misalignment'."""
        return self._scale_misalignment

    @scale_misalignment.setter
    def scale_misalignment(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'scale_misalignment' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'scale_misalignment' numpy.ndarray() must have a size of 9"
            self._scale_misalignment = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'scale_misalignment' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._scale_misalignment = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def temperature_slope(self):
        """Message field 'temperature_slope'."""
        return self._temperature_slope

    @temperature_slope.setter
    def temperature_slope(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'temperature_slope' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'temperature_slope' numpy.ndarray() must have a size of 9"
            self._temperature_slope = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'temperature_slope' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._temperature_slope = numpy.array(value, dtype=numpy.float64)
