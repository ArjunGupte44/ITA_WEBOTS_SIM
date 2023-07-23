# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/DiverseArray.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DiverseArray(type):
    """Metaclass of message 'DiverseArray'."""

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
            module = import_type_support('robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_interfaces.msg.DiverseArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__diverse_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__diverse_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__diverse_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg__diverse_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__diverse_array

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DiverseArray(metaclass=Metaclass_DiverseArray):
    """Message class 'DiverseArray'."""

    __slots__ = [
        '_robot_name',
        '_poi_x',
        '_poi_y',
        '_poi_z',
        '_arrival_time',
    ]

    _fields_and_field_types = {
        'robot_name': 'string',
        'poi_x': 'double',
        'poi_y': 'double',
        'poi_z': 'double',
        'arrival_time': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_name = kwargs.get('robot_name', str())
        self.poi_x = kwargs.get('poi_x', float())
        self.poi_y = kwargs.get('poi_y', float())
        self.poi_z = kwargs.get('poi_z', float())
        self.arrival_time = kwargs.get('arrival_time', float())

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
        if self.robot_name != other.robot_name:
            return False
        if self.poi_x != other.poi_x:
            return False
        if self.poi_y != other.poi_y:
            return False
        if self.poi_z != other.poi_z:
            return False
        if self.arrival_time != other.arrival_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_name(self):
        """Message field 'robot_name'."""
        return self._robot_name

    @robot_name.setter
    def robot_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_name' field must be of type 'str'"
        self._robot_name = value

    @builtins.property
    def poi_x(self):
        """Message field 'poi_x'."""
        return self._poi_x

    @poi_x.setter
    def poi_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'poi_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'poi_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._poi_x = value

    @builtins.property
    def poi_y(self):
        """Message field 'poi_y'."""
        return self._poi_y

    @poi_y.setter
    def poi_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'poi_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'poi_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._poi_y = value

    @builtins.property
    def poi_z(self):
        """Message field 'poi_z'."""
        return self._poi_z

    @poi_z.setter
    def poi_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'poi_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'poi_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._poi_z = value

    @builtins.property
    def arrival_time(self):
        """Message field 'arrival_time'."""
        return self._arrival_time

    @arrival_time.setter
    def arrival_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'arrival_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'arrival_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._arrival_time = value
