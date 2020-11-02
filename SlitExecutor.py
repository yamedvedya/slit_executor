#!/usr/bin/env python
# -*- coding:utf-8 -*-


##############################################################################
## license :
##============================================================================
##
## File :        SlitExecutor.py
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
##            This file is generated by POGO
##    (Program Obviously used to Generate tango Object)
##
##        (c) - Software Engineering Group - ESRF
##############################################################################

"""Executes virtual motor scripts"""

__all__ = ["SlitExecutor", "SlitExecutorClass", "main"]

__docformat__ = 'restructuredtext'

import PyTango
import sys
# Add additional import
# ----- PROTECTED REGION ID(VmExecutor.additionnal_import) ENABLED START -----#

# ----- PROTECTED REGION END -----#	//	VmExecutor.additionnal_import

## Device States Description
## ON :
## MOVING :
## FAULT :

class SlitExecutor(PyTango.Device_4Impl):

    # --------- Add you global variables here --------------------------
    # ----- PROTECTED REGION ID(VmExecutor.global_variables) ENABLED START -----#

    # ----- PROTECTED REGION END -----#	//	VmExecutor.global_variables

    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)
        self.debug_stream("In __init__()")
        SlitExecutor.init_device(self)
        # ----- PROTECTED REGION ID(VmExecutor.__init__) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.__init__

    def delete_device(self):
        self.debug_stream("In delete_device()")
        # ----- PROTECTED REGION ID(VmExecutor.delete_device) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.delete_device

    def init_device(self):
        self.debug_stream("In init_device()")
        self.get_device_properties(self.get_device_class())
        self.attr_Position_read = 0.0
        self.attr_CwLimit_read = 0
        self.attr_CcwLimit_read = 0
        self.attr_UnitLimitMin_read = 0.0
        self.attr_UnitLimitMax_read = 0.0
        self.attr_PositionSim_read = 0.0
        self.attr_ResultSim_read = ['']

        # --------------------------------------------------------
        # check what is our mode
        # --------------------------------------------------------

        if self.Direction == 'Horizontal':
            self.motor_names = ['Left', 'Right']
        elif self.Direction == 'Vertical':
            self.motor_names = ['Top', 'Bottom']
        else:
            PyTango.Except.throw_exception("vm", "Unknown mode", "VmExecutor")

        # --------------------------------------------------------
        # making real motor proxies
        # --------------------------------------------------------
        self.proxies = []
        for name in self.motor_names:
            try:
                proxy = getattr(self, name)
            except:
                PyTango.Except.throw_exception("vm", 'Cannot find {} attribute'.format(name), "VmExecutor")

            self.proxies.append(PyTango.DeviceProxy(proxy))
            self.result_sim.append('{} [{}]: position_sim not set'.format(name, proxy))


        # ----- PROTECTED REGION ID(VmExecutor.init_device) ENABLED START -----#
        self.set_state(PyTango.DevState.ON)

        # Create a proxy to the own device for setting Position properties
        self.dev = PyTango.DeviceProxy(self.get_name())

        # ----- PROTECTED REGION END -----#	//	VmExecutor.init_device

    def always_executed_hook(self):
        self.debug_stream("In always_excuted_hook()")
        # ----- PROTECTED REGION ID(VmExecutor.always_executed_hook) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.always_executed_hook

    # -----------------------------------------------------------------------------
    #    VmExecutor read/write attribute methods
    # -----------------------------------------------------------------------------

    def read_Position(self, attr):
        self.debug_stream("In read_Position()")
        # ----- PROTECTED REGION ID(VmExecutor.Position_read) ENABLED START -----#

        self.attr_Position_read = self._real_motors_to_vm()
        attr.set_value(self.attr_Position_read)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.Position_read

    def write_Position(self, attr):
        self.debug_stream("In write_Position()")
        new_position = attr.get_write_value()
        # ----- PROTECTED REGION ID(VmExecutor.Position_write) ENABLED START -----#

        min_value = self._get_limit_min()
        max_value = self._get_limit_max()

        if new_position < min_value or new_position > max_value:
            PyTango.Except.throw_exception("write_Position",
                                           "Position " + str(new_position) + " out of limits (min: " + str(
                                               min_value) + ", max: " + str(max_value) + ")",
                                           "VmExecutor")

        for motor_proxy, new_position in zip(self.proxies, self.vm_to_real_motors(new_position)):
            motor_proxy.Position = new_position

        return True

        # ----- PROTECTED REGION END -----#	//	VmExecutor.Position_write

    def read_CwLimit(self, attr):
        self.debug_stream("In read_CwLimit()")
        # ----- PROTECTED REGION ID(VmExecutor.CwLimit_read) ENABLED START -----#

        no_limit = True
        #
        # if one of the motors is in the limit return 1
        #
        for proxy in self.proxies:
            no_limit *= proxy.CwLimit == 0

        self.attr_CwLimit_read = no_limit
        attr.set_value(self.attr_CwLimit_read)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.CwLimit_read

    def read_CcwLimit(self, attr):
        self.debug_stream("In read_CcwLimit()")
        # ----- PROTECTED REGION ID(VmExecutor.CcwLimit_read) ENABLED START -----#
        no_limit = True
        #
        # if one of the motors is in the limit return 1
        #
        for proxy in self.proxies:
            no_limit *= proxy.CCwLimit == 0

        self.attr_CcwLimit_read = no_limit
        attr.set_value(self.attr_CcwLimit_read)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.CcwLimit_read

    def read_UnitLimitMin(self, attr):
        self.debug_stream("In read_UnitLimitMin()")
        # ----- PROTECTED REGION ID(VmExecutor.UnitLimitMin_read) ENABLED START -----#
        self.attr_UnitLimitMin_read = self._get_limit_min()

        cfg = []
        config = self.dev.get_attribute_config("Position")
        config.min_value = str(self.attr_UnitLimitMin_read)

        cfg.append(config)

        self.dev.set_attribute_config(cfg)

        attr.set_value(self.attr_UnitLimitMin_read)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.UnitLimitMin_read


    def read_UnitLimitMax(self, attr):
        self.debug_stream("In read_UnitLimitMax()")
        # ----- PROTECTED REGION ID(VmExecutor.UnitLimitMax_read) ENABLED START -----#

        self.attr_UnitLimitMax_read = self._get_limit_max()

        cfg = []
        config = self.dev.get_attribute_config("Position")
        config.max_value = str(self.attr_UnitLimitMax_read)

        cfg.append(config)

        self.dev.set_attribute_config(cfg)

        attr.set_value(self.attr_UnitLimitMax_read)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.UnitLimitMax_read


    def read_PositionSim(self, attr):
        self.debug_stream("In read_PositionSim()")
        # ----- PROTECTED REGION ID(VmExecutor.PositionSim_read) ENABLED START -----#
        attr.set_value(self.attr_PositionSim_read)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.PositionSim_read

    def write_PositionSim(self, attr):
        self.debug_stream("In write_PositionSim()")
        # ----- PROTECTED REGION ID(VmExecutor.PositionSim_write) ENABLED START -----#
        self.attr_PositionSim_read = attr.get_write_value()
        # ----- PROTECTED REGION END -----#	//	VmExecutor.PositionSim_write

    def read_ResultSim(self, attr):
        self.debug_stream("In read_ResultSim()")
        # ----- PROTECTED REGION ID(VmExecutor.ResultSim_read) ENABLED START -----#
        _answer = []
        for name, position in zip(self.motor_names, self.vm_to_real_motors(self.position_sim)):
            _answer.append("{} [{}]: {}".format(name, getattr(self, name), position))

        self.attr_ResultSim_read = _answer
        attr.set_value(self.attr_ResultSim_read, len(self.attr_ResultSim_read))

        # ----- PROTECTED REGION END -----#	//	VmExecutor.ResultSim_read

        # ----- PROTECTED REGION ID(VmExecutor.initialize_dynamic_attributes) ENABLED START -----#

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

        # ----- PROTECTED REGION END -----#	//	VmExecutor.initialize_dynamic_attributes

    def read_attr_hardware(self, data):
        self.debug_stream("In read_attr_hardware()")
        # ----- PROTECTED REGION ID(VmExecutor.read_attr_hardware) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.read_attr_hardware

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
        # ----- PROTECTED REGION ID(VmExecutor.State) ENABLED START -----#

        #
        # if one device is in FAULT the VM is in FAULT too
        #
        for proxy in self.proxies:
            if proxy.state() == PyTango.DevState.FAULT:
                argout = PyTango.DevState.FAULT
                break
        if argout == PyTango.DevState.ON:
            #
            # if one device is MOVING the VM is MOVING too
            #
            for proxy in self.proxies:
                if proxy.state() == PyTango.DevState.MOVING:
                    argout = PyTango.DevState.MOVING
                    break

        self.set_state(argout)

        # ----- PROTECTED REGION END -----#	//	VmExecutor.State
        if argout != PyTango.DevState.ALARM:
            PyTango.Device_4Impl.dev_state(self)
        return self.get_state()

    def Calibrate(self, argin):
        """ Re-defines the current position by changing the HomePosition, UnitCalibration, etc.

        :param argin:
        :type: PyTango.DevDouble
        :return:
        :rtype: PyTango.DevLong """
        self.debug_stream("In Calibrate()")
        # ----- PROTECTED REGION ID(VmExecutor.Calibrate) ENABLED START -----#

        try:
            for proxy, position in zip(self.proxies, self.vm_to_real_motors(argin)):
                proxy.Calibrate(position)
            return True
        except:
            return False

        # ----- PROTECTED REGION END -----#	//	VmExecutor.Calibrate
        return argout

    def is_Calibrate_allowed(self):
        self.debug_stream("In is_Calibrate_allowed()")
        state_ok = not (self.get_state() in [PyTango.DevState.MOVING,
                                             PyTango.DevState.FAULT])
        # ----- PROTECTED REGION ID(VmExecutor.is_Calibrate_allowed) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.is_Calibrate_allowed
        return state_ok

    def StopMove(self):
        """

        :param :
        :type: PyTango.DevVoid
        :return:
        :rtype: PyTango.DevVoid """
        self.debug_stream("In StopMove()")
        # ----- PROTECTED REGION ID(VmExecutor.StopMove) ENABLED START -----#
        for proxy in self.proxies:
            proxy.StopMove()

        # ----- PROTECTED REGION END -----#	//	VmExecutor.StopMove

    def read_DynAttr(self, attr):
        value = self.vm.read_DynamicAttr(attr.get_name())
        attr.set_value(value)

    def write_DynAttr(self, attr):
        self.vm.write_DynamicAttr(attr.get_name(), attr.get_write_value())

    # ----- PROTECTED REGION END -----#	//	VmExecutor.programmer_methods


    # --------------------------------------------------------
    # real_motors_to_vm
    # --------------------------------------------------------

    def _real_motors_to_vm(self):
        ###
        # this function returns the position of slit according to the current mode
        ###
        p1 = self.proxies[0].Position
        p2 = self.proxies[1].Position

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
            delta = (new_position - self.real_motors_to_vm())/2.
            return self.proxies[0].Position + delta, self.proxies[1].Position + delta

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            positions = []
            delta = new_position - self.real_motors_to_vm()
            for proxy in self.proxies:
                positions.append(proxy.Position + delta)
            return positions

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
            distances_to_limit = (self.proxies[0].UnitLimitMax - self.proxies[0].Position,
                                  self.proxies[1].Position - self.proxies[1].UnitLimitMin)

            if distances_to_limit[0] != distances_to_limit[0]:
                limit_max = self.real_motors_to_vm() + 2 * min(distances_to_limit)
            else:
                limit_max = self.proxies[0].UnitLimitMax - self.proxies[1].UnitLimitMin
            return limit_max

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            distance_to_limit = []
            for proxy in self.proxies:
                distance_to_limit.append(proxy.UnitLimitMax - proxy.Position)

            return self.real_motors_to_vm() + min(distance_to_limit)

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
            distances_to_limit = (self.proxies[0].Position - self.proxies[0].UnitLimitMin,
                                  self.proxies[1].UnitLimitMax - self.proxies[1].Position)

            if distances_to_limit[0] != distances_to_limit[0]:
                limit_min = self.real_motors_to_vm() - 2 * min(distances_to_limit)
            else:
                limit_min = self.proxies[0].UnitLimitMin - self.proxies[1].UnitLimitMax
            return limit_min

        elif str(self.Mode).lower() in ['p', 'pos', 'position']:
            distance_to_limit = []
            for proxy in self.proxies:
                distance_to_limit.append(proxy.UnitLimitMin - proxy.Position)

            return self.real_motors_to_vm() + min(distance_to_limit)

        else:
            PyTango.Except.throw_exception("slit", "Unknown mode", "SlitExecutor")


class SlitExecutorClass(PyTango.DeviceClass):
    # --------- Add you global class variables here --------------------------
    # ----- PROTECTED REGION ID(VmExecutor.global_class_variables) ENABLED START -----#

    # ----- PROTECTED REGION END -----#	//	VmExecutor.global_class_variables

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
        # ----- PROTECTED REGION ID(VmExecutor.dyn_attr) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.dyn_attr

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
              PyTango.READ, 100]],
    }


def main():
    try:
        py = PyTango.Util(sys.argv)
        py.add_class(SlitExecutorClass, SlitExecutor, 'SlitExecutor')
        # ----- PROTECTED REGION ID(VmExecutor.add_classes) ENABLED START -----#

        # ----- PROTECTED REGION END -----#	//	VmExecutor.add_classes

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed as e:
        print('-------> Received a DevFailed exception: %s ' % repr(e))
    except Exception as e:
        print('-------> An unforeseen exception occured.... %s ' % repr(e))


if __name__ == '__main__':
    main()
