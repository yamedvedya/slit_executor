#!/usr/bin/env python
# -*- coding:utf-8 -*-


##############################################################################
## license :
##============================================================================
##
## File :        CombinedMotor.py
##
## Project :     TANGO Device Server
##
## This file is part of Tango device class.
##
## Tango is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Tango is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Tango.  If not, see <http://www.gnu.org/licenses/>.
##
##
## $Author :      yury.matveev@desy.de
##
## $Revision :    $
##
## $Date :        $
##
## $HeadUrl :     $
##============================================================================

"""
Use to do coupled motion of several motors by one combined motor (VM)

To setup full the MOTOR array with (<motor>, <coupling coefficient>, <position coefficient>) tuples, where:

<motor> - tango address of motor

<coupling coefficients> - how the position, speed, acceleration of physical motor scaled relative to the CM properties
i.e.:
    physical motor position = coupling coefficient * combined motor position

<position coefficient> - coefficient for calculation of combined motor property out of physical motors
i.e.:
    combined motor position = motor1 * position coefficient1 + motor2 * position coefficient2 + ....

Important:
both coefficients must obey condition:
1 = coupling coefficients1 * position coefficient1 + coupling coefficients2 * position coefficient2 + ...

Examples:
    For slit center (cy) assuming that  slit_top, slit_bottom moves positive up
        coupling coefficient 1 = 1
        coupling coefficient 2 = 1

        position coefficient 1 = 0.5
        position coefficient 2 = 0.5

    For slit gap (dy) assuming slit_top, slit_bottom moves positive up
        coupling coefficient 1 = 0.5
        coupling coefficient 2 = -0.5

        position coefficient 1 = 1
        position coefficient 2 = -1

    For table with 3 legs y1, y2, y3 using 1st leg to define value of virtual motor
        coupling coefficient 1 = 1
        coupling coefficient 2 = 1
        coupling coefficient 3 = 1

        position coefficient 1 = 1
        position coefficient 2 = 0
        position coefficient 3 = 0

    For theta 2 theta motor with th, 2th - real motors
        coupling coefficient 1 = 1
        coupling coefficient 2 = 2

        position coefficient 1 = 1
        position coefficient 2 = 0


Limits for motion taken correctly from soft limits of physical motors.
Change of limits of virtual motor is not allowed

Calibration of the position of the virtual motor is allowed and done for
via calibration of physical motors using COEF (like motion).
"""

__all__ = ["CombinedMotor", "CombinedMotorClass", "main"]

__docformat__ = 'restructuredtext'


# here we define which attributes has to scale, and which - not

COMMON_ATTRIBUTES = {'Acceleration': True,
                     'BaseRate': True,
                     'Conversion': False,
                     'SlewRate': True,
                     'SlewRateMax': False,
                     'SlewRateMin': False,
                     'StepBacklash': False}

import PyTango
import sys
import os
import importlib
import numpy as np

class CombinedMotor(PyTango.Device_4Impl):

    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.debug_stream("In __init__()")
        CombinedMotor.init_device(self)

    def delete_device(self):
        self.debug_stream("In delete_device()")

    def init_device(self):
        self.debug_stream("In init_device()")
        self.get_device_properties(self.get_device_class())
        self._position_sim = 0.0

        sys.path.append(os.path.dirname(self.MotorsCode))
        basename = os.path.basename(self.MotorsCode)

        #
        # remove the extension .py
        #
        if basename.find(".py") > 0:
            basename = basename.rpartition('.')[0]

        try:
            _motors_definition = importlib.import_module(basename).MOTORS
        except:
            PyTango.Except.throw_exception("vm", 'Cannot import MOTORS from {}'.format(basename), "CombinedMotor")


        # --------------------------------------------------------
        # making real motor proxies
        # --------------------------------------------------------
        self._motors = []
        for name, coupling, position in self._motors:
            try:
                self._motors.append((PyTango.DeviceProxy(name), coupling, position))
            except:
                PyTango.Except.throw_exception("vm", 'Cannot find {} motor'.format(name), "VmExecutor")

        self.set_state(PyTango.DevState.ON)

        # Create a proxy to the own device for setting Position properties
        self.dev = PyTango.DeviceProxy(self.get_name())


        # Checking whether sub-motors have equal settings

        for attribute in COMMON_ATTRIBUTES.keys():
            self._get_attribute(attribute)

    def always_executed_hook(self):
        self.debug_stream("In always_excuted_hook()")

    # -----------------------------------------------------------------------------
    #    Motor related read/write attribute methods
    # -----------------------------------------------------------------------------

    def read_Position(self, attr):

        self.debug_stream("In read_Position()")
        attr.set_value(self._real_motors_to_vm())


    # -----------------------------------------------------------------------------
    def write_Position(self, attr):

        self.debug_stream("In write_Position()")
        new_position = attr.get_write_value()

        min_value = self._get_limit_min()
        max_value = self._get_limit_max()

        if new_position < min_value or new_position > max_value:
            PyTango.Except.throw_exception("write_Position",
                                           "Position " + str(new_position) + " out of limits (min: " + str(
                                               min_value) + ", max: " + str(max_value) + ")",
                                           "VmExecutor")

        for motor_proxy, new_position in zip(self._motors, self._vm_to_real_motors(new_position)):
            motor_proxy.Position = new_position


    # -----------------------------------------------------------------------------
    def read_CwLimit(self, attr):

        self.debug_stream("In read_CwLimit()")

        no_limit = True
        #
        # if one of the motors is in the limit return 1
        #
        for proxy in self._motors:
            no_limit *= proxy.CwLimit == 0

        attr.set_value(not no_limit)


    # -----------------------------------------------------------------------------
    def read_CcwLimit(self, attr):

        self.debug_stream("In read_CcwLimit()")
        no_limit = True
        #
        # if one of the motors is in the limit return 1
        #
        for proxy in self._motors:
            no_limit *= proxy.CCwLimit == 0

        attr.set_value(not no_limit)


    # -----------------------------------------------------------------------------
    def read_UnitLimitMin(self, attr):

        self.debug_stream("In read_UnitLimitMin()")
        self.attr_UnitLimitMin_read = self._get_limit_min()

        cfg = []
        config = self.dev.get_attribute_config("Position")
        config.min_value = str(self.attr_UnitLimitMin_read)

        cfg.append(config)

        self.dev.set_attribute_config(cfg)

        attr.set_value(self.attr_UnitLimitMin_read)


    # -----------------------------------------------------------------------------
    def read_UnitLimitMax(self, attr):

        self.debug_stream("In read_UnitLimitMax()")

        unit_limit_max = self._get_limit_max()

        cfg = []
        config = self.dev.get_attribute_config("Position")
        config.max_value = str(unit_limit_max)

        cfg.append(config)

        self.dev.set_attribute_config(cfg)

        attr.set_value(unit_limit_max)

    # -----------------------------------------------------------------------------
    def read_PositionSim(self, attr):

        self.debug_stream("In read_PositionSim()")
        attr.set_value(self._position_sim)


    # -----------------------------------------------------------------------------
    def write_PositionSim(self, attr):

        self.debug_stream("In write_PositionSim()")
        self._position_sim = attr.get_write_value()


    # -----------------------------------------------------------------------------
    def read_ResultSim(self, attr):

        self.debug_stream("In read_ResultSim()")
        _answer = []
        for name, position in zip(self._motor_names, self._vm_to_real_motors(self._position_sim)):
            _answer.append("{} [{}]: {}".format(name, getattr(self, name), position))

        attr.set_value(_answer, len(_answer))

    # -----------------------------------------------------------------------------
    #    These read/write attribute methods needed for speed and acceleration control
    # -----------------------------------------------------------------------------

    def _set_attribute(self, name, value):

        for proxy in self._motors:
            setattr(proxy, name, value*np.sign(getattr(proxy, name)))

    # -----------------------------------------------------------------------------
    def _get_attribute(self, name):
        # first we need to check that attribute values are the same for all motors,
        # set it to min value (maintaining the sign!!) if not, and only then return absolute (!!) value

        values = [getattr(proxy, name)*scale if COMMON_ATTRIBUTES[name] else getattr(proxy, name)
                  for proxy, _, scale in self._motors]

        new_value = np.min(np.abs(values))
        for proxy, _, _, value in zip(self._motors, values):
            setattr(proxy, name, new_value*np.sign(value))

        return new_value

    # -----------------------------------------------------------------------------
    def read_Acceleration(self, attr):

        self.debug_stream("In read_Acceleration()")
        attr.set_value(self._get_attribute('Acceleration'))

    # -----------------------------------------------------------------------------
    def write_Acceleration(self, attr):

        self.debug_stream("In write_Acceleration()")
        self._set_attribute('Acceleration', attr.get_write_value())
        
    # -----------------------------------------------------------------------------
    def read_BaseRate(self, attr):

        self.debug_stream("In read_BaseRate()")
        attr.set_value(self._get_attribute('BaseRate'))

    # -----------------------------------------------------------------------------
    def write_BaseRate(self, attr):

        self.debug_stream("In write_BaseRate()")
        self._set_attribute('BaseRate', attr.get_write_value())

    # -----------------------------------------------------------------------------
    def read_Conversion(self, attr):

        self.debug_stream("In read_Conversion()")
        attr.set_value(self._get_attribute('Conversion'))

    # -----------------------------------------------------------------------------
    def write_Conversion(self, attr):

        self.debug_stream("In write_Conversion()")
        self._set_attribute('Conversion', attr.get_write_value())
        
    # -----------------------------------------------------------------------------
    def read_SlewRate(self, attr):

        self.debug_stream("In read_SlewRate()")
        attr.set_value(self._get_attribute('SlewRate'))

    # -----------------------------------------------------------------------------
    def write_SlewRate(self, attr):

        self.debug_stream("In write_SlewRate()")
        self._set_attribute('SlewRate', attr.get_write_value())
        
    # -----------------------------------------------------------------------------
    def read_SlewRateMax(self, attr):

        self.debug_stream("In read_SlewRateMax()")
        attr.set_value(self._get_attribute('SlewRateMax'))

    # -----------------------------------------------------------------------------
    def write_SlewRateMax(self, attr):

        self.debug_stream("In write_SlewRateMax()")
        self._set_attribute('SlewRateMax', attr.get_write_value())
        
    # -----------------------------------------------------------------------------
    def read_SlewRateMin(self, attr):

        self.debug_stream("In read_SlewRateMin()")
        attr.set_value(self._get_attribute('SlewRateMin'))

    # -----------------------------------------------------------------------------
    def write_SlewRateMin(self, attr):

        self.debug_stream("In write_SlewRateMin()")
        self._set_attribute('SlewRateMin', attr.get_write_value())
        
    # -----------------------------------------------------------------------------
    def read_StepBacklash(self, attr):

        self.debug_stream("In read_StepBacklash()")
        attr.set_value(self._get_attribute('StepBacklash'))

    # -----------------------------------------------------------------------------
    def write_StepBacklash(self, attr):

        self.debug_stream("In write_StepBacklash()")
        self._set_attribute('StepBacklash', attr.get_write_value())

    # -----------------------------------------------------------------------------
    #    Support of dynamic attribute
    # -----------------------------------------------------------------------------

    # -----------------------------------------------------------------------------
    def initialize_dynamic_attributes(self):
        for attr in self.DynamicAttributes:
            attr = attr.replace("\"", "")
            attr = attr.split(',')
            name = attr[0]
            if len(attr) > 1:
                typ = attr[1]
            else:
                typ = None
            if len(attr) > 2:
                acc = attr[2]
            else:
                acc = None
            if len(attr) > 3:
                dimension = attr[3]
            else:
                dimension = 0
            if typ == "double":
                typ = PyTango.DevDouble
            elif typ == "string":
                typ = PyTango.DevString
            else:
                typ = PyTango.DevLong
            if acc == "r":
                acc = PyTango.READ
            else:
                acc = PyTango.READ_WRITE
            if dimension < 2:
                pt_attr = PyTango.Attr(name, typ, acc)
            else:
                pt_attr = PyTango.SpectrumAttr(name, typ, acc, int(dimension))

            self.add_attribute(pt_attr, self.read_DynAttr, self.write_DynAttr)

    # -----------------------------------------------------------------------------
    def read_DynAttr(self, attr):
        value = self.vm.read_DynamicAttr(attr.get_name())
        attr.set_value(value)

    # -----------------------------------------------------------------------------
    def write_DynAttr(self, attr):
        self.vm.write_DynamicAttr(attr.get_name(), attr.get_write_value())

    # -----------------------------------------------------------------------------
    def read_attr_hardware(self, data):
        self.debug_stream("In read_attr_hardware()")

    # -----------------------------------------------------------------------------
    #    VmExecutor command methods
    # -----------------------------------------------------------------------------

    def dev_state(self):
        """ This command gets the device state (stored in its <i>device_state</i> data member) and returns it to the caller.

        :param : none.
        :type: PyTango.DevVoid
        :return: State Code
        :rtype: PyTango.CmdArgType.DevState """

        self.debug_stream("In dev_state()")
        argout = PyTango.DevState.ON

        #
        # if one device is in FAULT the VM is in FAULT too
        #
        for proxy in self._motors:
            if proxy.state() == PyTango.DevState.FAULT:
                argout = PyTango.DevState.FAULT
                break
        if argout == PyTango.DevState.ON:
            #
            # if one device is MOVING the VM is MOVING too
            #
            for proxy in self._motors:
                if proxy.state() == PyTango.DevState.MOVING:
                    argout = PyTango.DevState.MOVING
                    break

        self.set_state(argout)

        if argout != PyTango.DevState.ALARM:
            PyTango.Device_4Impl.dev_state(self)
        return self.get_state()

    # -----------------------------------------------------------------------------
    def Calibrate(self, argin):
        """ Re-defines the current position by changing the HomePosition, UnitCalibration, etc.

        :param argin:
        :type: PyTango.DevDouble
        :return:
        :rtype: PyTango.DevLong """

        self.debug_stream("In Calibrate()")
        try:
            for proxy, position in zip(self._motors, self.vm_to_real_motors(argin)):
                proxy.Calibrate(position)
            return True
        except:
            return False

    # -----------------------------------------------------------------------------
    def is_Calibrate_allowed(self):
        self.debug_stream("In is_Calibrate_allowed()")
        state_ok = not (self.get_state() in [PyTango.DevState.MOVING,
                                             PyTango.DevState.FAULT])
        return state_ok

    # -----------------------------------------------------------------------------
    def StopMove(self):
        """

        :param :
        :type: PyTango.DevVoid
        :return:
        :rtype: PyTango.DevVoid """
        self.debug_stream("In StopMove()")
        for proxy in self._motors:
            proxy.StopMove()

    # --------------------------------------------------------
    # real_motors_to_vm
    # --------------------------------------------------------

    def _real_motors_to_vm(self):
        ###
        # this function returns the position of slit according to the current mode
        ###
        p1 = self._motors[0].Position
        p2 = self._motors[1].Position

        if str(self.Mode).lower() in ['g', 'gap']:
            return p1 - p2

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            return (p1 + p2) / 2.

        else:
            PyTango.Except.throw_exception("slit", "Unknown mode", "SlitExecutor")


    # --------------------------------------------------------
    # vm_to_real_motors
    # --------------------------------------------------------

    def _vm_to_real_motors(self, new_position):
        ###
        # this function returns the position real motors from the position of slits according to the current mode
        ###

        if str(self.Mode).lower() in ['g', 'gap']:
            delta = (new_position - self._real_motors_to_vm())/2.
            return self._motors[0].Position + delta, self._motors[1].Position - delta

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            delta = new_position - self._real_motors_to_vm()
            return self._motors[0].Position + delta, self._motors[1].Position + delta
        else:
            PyTango.Except.throw_exception("slit", "Unknown mode", "SlitExecutor")

    # --------------------------------------------------------
    # _get_limit_max
    # --------------------------------------------------------

    def _get_limit_max(self):
        ###
        # this function returns the max limits of slits according to the current mode
        ###
        if str(self.Mode).lower() in ['g', 'gap']:
            distances_to_limit = (self._motors[0].UnitLimitMax - self._motors[0].Position,
                                  self._motors[1].Position - self._motors[1].UnitLimitMin)

            if distances_to_limit[0] != distances_to_limit[1]:
                limit_max = self._real_motors_to_vm() + 2 * min(distances_to_limit)
            else:
                limit_max = self._motors[0].UnitLimitMax - self._motors[1].UnitLimitMin
            return limit_max

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            distance_to_limit = []
            for proxy in self._motors:
                distance_to_limit.append(proxy.UnitLimitMax - proxy.Position)

            return self._real_motors_to_vm() + min(distance_to_limit)

        else:
            PyTango.Except.throw_exception("slit", "Unknown mode", "SlitExecutor")

    # --------------------------------------------------------
    # _get_limit_max
    # --------------------------------------------------------

    def _get_limit_min(self):
        ###
        # this function returns the max limits of slits according to the current mode
        ###
        if str(self.Mode).lower() in ['g', 'gap']:
            distances_to_limit = (self._motors[0].Position - self._motors[0].UnitLimitMin,
                                  self._motors[1].UnitLimitMax - self._motors[1].Position)

            if distances_to_limit[0] != distances_to_limit[1]:
                limit_min = self._real_motors_to_vm() - 2 * min(distances_to_limit)
            else:
                limit_min = self._motors[0].UnitLimitMin - self._motors[1].UnitLimitMax
            return limit_min

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            distance_to_limit = []
            for proxy in self._motors:
                distance_to_limit.append(proxy.UnitLimitMin - proxy.Position)

            return self._real_motors_to_vm() + min(distance_to_limit)

        else:
            PyTango.Except.throw_exception("slit", "Unknown mode", "SlitExecutor")




class CombinedMotorClass(PyTango.DeviceClass):

    def dyn_attr(self, dev_list):
        """Invoked to create dynamic attributes for the given devices.
        Default implementation calls
        :meth:`VmExecutor.initialize_dynamic_attributes` for each device

        :param dev_list: list of devices
        :type dev_list: :class:`PyTango.DeviceImpl`"""

        for dev in dev_list:
            try:
                dev.initialize_dynamic_attributes()
            except:
                import traceback
                dev.warn_stream("Failed to initialize dynamic attributes")
                dev.debug_stream("Details: " + traceback.format_exc())

    #    Class Properties
    class_property_list = {
    }

    #    Device Properties
    device_property_list = {
        'Direction':
            [PyTango.DevString,
             "The direction of movement (Horizontal/Vertical)",
             ["None"]],
        'Mode':
            [PyTango.DevString,
             "The movement mode (Gap/Position)",
             ["None"]],
        'Left':
            [PyTango.DevString,
             "Motor, controlling left slit",
             ["None"]],
        'Right':
            [PyTango.DevString,
             "Motor, controlling right slit",
             ["None"]],
        'Top':
            [PyTango.DevString,
             "Motor, controlling top slit",
             ["None"]],
        'Bottom':
            [PyTango.DevString,
             "Motor, controlling bottom slit",
             ["None"]],
        'DynamicAttributes':
            [PyTango.DevVarStringArray,
             "Array of strings: AttributeName, type, rd",
             [None]],
    }

    #    Command definitions
    cmd_list = {
        'Calibrate':
            [[PyTango.DevDouble, "none"],
             [PyTango.DevLong, "none"]],
        'StopMove':
            [[PyTango.DevVoid, "none"],
             [PyTango.DevVoid, "none"]],
    }

    #    Attribute definitions
    attr_list = {
        'Position':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'CwLimit':
            [[PyTango.DevLong,
              PyTango.SCALAR,
              PyTango.READ]],
        'CcwLimit':
            [[PyTango.DevLong,
              PyTango.SCALAR,
              PyTango.READ]],
        'UnitLimitMin':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ]],
        'UnitLimitMax':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ]],
        'PositionSim':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'ResultSim':
            [[PyTango.DevString,
              PyTango.SPECTRUM,
              PyTango.READ, 2]],

        # these properties are needed for speed control

        'Conversion':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'Acceleration':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'BaseRate':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'SlewRate':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'SlewRateMax':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'SlewRateMin':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
        'StepBacklash':
            [[PyTango.DevDouble,
              PyTango.SCALAR,
              PyTango.READ_WRITE]],
    }


def main():
    try:
        py = PyTango.Util(sys.argv)
        py.add_class(CombinedMotorClass, CombinedMotor, 'CombinedMotor')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed as e:
        print('-------> Received a DevFailed exception: %s ' % repr(e))
    except Exception as e:
        print('-------> An unforeseen exception occured.... %s ' % repr(e))


if __name__ == '__main__':
    main()
