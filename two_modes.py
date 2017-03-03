# _____________________________________________ TWO MODES __________________________________________________ #

import time
from libs import communications, filterings, datalog
from pygame import mixer
import numpy as np
import threading
from pyardrone import ARDrone


def input_thread(line):
    input()
    line.append(None)


class FreeMovement:
    communication = communications.CommunicationManager()
    filtering = filterings.FilteringManager()
    report = datalog.DatalogManager()
    drone = ARDrone()
    drone.emergency()
    drone.trim()
    drone.navdata_ready.wait()  # wait until NavData is ready
    drone.set_navdata_available()
    navdata = drone.get_navdata()
    initial_yaw = navdata.psi
    bat = navdata.vbat_flying_percentage
    print('Battery = '+ str(bat)+'%')

    mixer.init()
    alert1 = mixer.Sound('beep-07.wav')
    alert2 = mixer.Sound('beep-08.wav')

    communication.open_serial_port()
    max_samples = 14
    watch_samples_counter = -1

    handside = 'left'

    x_axis_acceleration = []
    y_axis_acceleration = []
    z_axis_acceleration = []
    acceleration = []
    clap = 0
    sample_time=0.05
    time_limit = 120

    # Datalog time

    report.create_file('detection.txt')
    report.record_data('detection.txt', 'direction', 'x', 'y', 'z')
    mode=[0]
    line = []
    threading._start_new_thread(input_thread, (line,))

    x_direction = '    '
    old_x_direction = '    '
    y_direction = '    '
    old_y_direction = '    '

    take_off = False

    time_initial = time.time()
    last_activity = time_initial
    last_down_movement = 0
    while time.time() - time_initial <= time_limit:
        if line:
            break
        if drone.state.vbat_low==1:
            print('low bat')
            break

        bytes_to_read = communication.send_data_request()
        inbyte = communication.read_data(bytes_to_read)
        enter = filtering.data_availability(bytes_to_read, inbyte)

        if (bytes_to_read >= 7 and inbyte[3] == 1 and enter is True) or (
                            bytes_to_read == 14 and inbyte[10] == 1 and enter is True):
            watch_samples_counter += 1

            if watch_samples_counter == 0:
                alert1.play()

            x_axis_acceleration.append(inbyte[bytes_to_read - 3])
            y_axis_acceleration.append(inbyte[bytes_to_read - 2])
            z_axis_acceleration.append(inbyte[bytes_to_read - 1])

            filtering.filter_acceleration(x_axis_acceleration, watch_samples_counter)
            filtering.filter_acceleration(y_axis_acceleration, watch_samples_counter)
            filtering.filter_acceleration(z_axis_acceleration, watch_samples_counter)

            x_axis_acceleration[watch_samples_counter
            ], y_axis_acceleration[watch_samples_counter
            ], z_axis_acceleration[watch_samples_counter] = filtering.handside_mode_3_axis(
                                                                            x_axis_acceleration[watch_samples_counter],
                                                                            y_axis_acceleration[watch_samples_counter],
                                                                            z_axis_acceleration[watch_samples_counter],
                                                                            handside)

            acceleration = [x_axis_acceleration[watch_samples_counter],
                            y_axis_acceleration[watch_samples_counter],
                            z_axis_acceleration[watch_samples_counter]]

            if np.linalg.norm(acceleration) >= 130 and time.time()-last_activity <= 3 and take_off is False:
                print('CLAP!!!')
                time.sleep(1)
                alert2.play()
                if clap == 1:
                    take_off = True
                    while not drone.state.fly_mask:
                        drone.takeoff()
                    drone.hover()
                    time.sleep(10)

                    alert1.play()
                    x_axis_acceleration[watch_samples_counter] = 15
                    y_axis_acceleration[watch_samples_counter] = 15

                    sample_time = 0.5
                    time.sleep(1)
                clap += 1

            mode.append(0)

            if take_off is True and x_axis_acceleration[watch_samples_counter] >= -85 \
                    and x_axis_acceleration[watch_samples_counter] <= -55:
                mode[watch_samples_counter] = 1
                print('mode = 1 (left...)')

            elif take_off is True and z_axis_acceleration[watch_samples_counter] >= 55 \
                    and z_axis_acceleration[watch_samples_counter] <= 85:
                mode[watch_samples_counter] = 2
                print('mode = 2 (up...)')
            if mode[watch_samples_counter - 1] != mode[watch_samples_counter]:
                print('MODE CHANGE')

            if mode[watch_samples_counter-2] == 1 and take_off is True \
                    and x_axis_acceleration[watch_samples_counter] <= -20 \
                    and x_axis_acceleration[watch_samples_counter] >= -55 and abs(
                    y_axis_acceleration[watch_samples_counter]) < abs(
                    x_axis_acceleration[watch_samples_counter]) and old_x_direction == '    '\
                    and z_axis_acceleration[watch_samples_counter] < -20:
                x_direction = 'left '
                alert1.play()
                print(x_direction)
                initial_flight_time = time.time()
                while time.time() - initial_flight_time <= 2:
                    drone.move(left=0.1)
                last_activity = time.time()

            elif mode[watch_samples_counter-2] == 1 and take_off is True \
                    and x_axis_acceleration[watch_samples_counter] <= -20 \
                    and x_axis_acceleration[watch_samples_counter] >= -55 and abs(
                    y_axis_acceleration[watch_samples_counter]) < abs(
                    x_axis_acceleration[watch_samples_counter]) and old_x_direction == '    ' \
                    and z_axis_acceleration[watch_samples_counter] > 20:
                x_direction = 'right '
                alert1.play()
                print(x_direction)
                initial_flight_time = time.time()
                while time.time() - initial_flight_time <= 2:
                    drone.move(right=0.1)
                last_activity = time.time()

            elif mode[watch_samples_counter-2] == 1 and take_off is True \
                    and y_axis_acceleration[watch_samples_counter] >= 30 and abs(
                    x_axis_acceleration[watch_samples_counter]) < abs(
                    y_axis_acceleration[watch_samples_counter]) and old_y_direction == '    ':
                y_direction = 'backward'
                alert1.play()
                print(y_direction)
                initial_flight_time = time.time()
                while time.time() - initial_flight_time <= 2:
                    drone.move(backward=0.1)
                last_activity = time.time()

            elif mode[watch_samples_counter-2] == 1 and take_off is True \
                    and y_axis_acceleration[watch_samples_counter] <= -20 and abs(
                    x_axis_acceleration[watch_samples_counter]) < abs(
                    y_axis_acceleration[watch_samples_counter]) and old_y_direction == '    ':
                y_direction = 'forward'
                alert1.play()
                print(y_direction)
                initial_flight_time = time.time()
                while time.time() - initial_flight_time <= 2:
                    drone.move(forward=0.1)
                last_activity = time.time()

            elif mode[watch_samples_counter-2] == 2 and take_off is True \
                    and y_axis_acceleration[watch_samples_counter] >= 30 and abs(
                    x_axis_acceleration[watch_samples_counter]) < abs(
                    y_axis_acceleration[watch_samples_counter]) and old_y_direction == '    ':
                y_direction = 'down'

                alert1.play()
                print(y_direction)
                if time.time() - last_down_movement <= 5:
                    time_initial = 0
                initial_flight_time = time.time()
                while time.time() - initial_flight_time <= 2:
                    drone.move(down=0.3)

                last_activity = time.time()
                last_down_movement = time.time()

            elif mode[watch_samples_counter-2] == 2 and take_off is True \
                    and y_axis_acceleration[watch_samples_counter] <= -20 and abs(
                    x_axis_acceleration[watch_samples_counter]) < abs(
                    y_axis_acceleration[watch_samples_counter]) and old_y_direction == '    ':
                y_direction = 'up'

                alert1.play()
                print(y_direction)
                initial_flight_time = time.time()
                while time.time() - initial_flight_time <= 2:
                    drone.move(up=0.3)
                last_activity = time.time()

            if time.time() - last_activity >= 1:
                last_activity = time.time()
                print('1 sec without movement')

            report.record_data('detection.txt', str(mode[watch_samples_counter]) + x_direction + y_direction,
                               x_axis_acceleration[watch_samples_counter],
                               y_axis_acceleration[watch_samples_counter],
                               z_axis_acceleration[watch_samples_counter])

            old_x_direction = x_direction
            x_direction = '    '
            old_y_direction = y_direction
            y_direction = '    '

            time.sleep(sample_time)

    alert2.play()

    time.sleep(1)

    navdata = drone.get_navdata()
    bat = navdata.vbat_flying_percentage
    print('Battery = ' + str(bat)+'%')

    while drone.state.fly_mask:
        drone.land()

    if not drone.state.fly_mask:
        print('CIAO!')

    # communication.close_serial_port()