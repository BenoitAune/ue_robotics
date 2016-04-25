import itertools
import time
import numpy
import pypot.dynamixel


if __name__ == '__main__':

    # we first open the Dynamixel serial port
    with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:

        # we can scan the motors
        # found_ids = dxl_io.scan()  # this may take several seconds
        # print 'Detected:', found_ids

       
        
        # we power on the motors
        dxl_io.enable_torque([10,11,12])

        # we get the current positions
        print 'Current pos:', dxl_io.get_present_position([10,11,12])

        for i in range(0,50):

            # we create a python dictionnary: {id0 : position0, id1 : position1...}
            pos = dict(zip([10,11,12],[10*numpy.sin(2*numpy.pi*0.5*time.time()),10*numpy.sin(2*numpy.pi*0.5*time.time()),10*numpy.sin(2*numpy.pi*0.5*time.time())]))
            print 'Cmd:', pos

            # we send these new positions
            dxl_io.set_goal_position(pos)
            time.sleep(0.1)  # we wait for 0.1s

        # we get the current positions
        print 'New pos:', dxl_io.get_present_position([10,11,12])

        # we power off the motors
        dxl_io.disable_torque([10,11,12])
        time.sleep(1)  # we wait for 1s
