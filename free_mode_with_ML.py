# free_mode_with_ML.py
# Mónica Milán (@mncmilan)
# mncmilan@gmail.com
# http://steelhummingbird.blogspot.com.es/

# This code obtains acceleration data from eZ430-Chronos watch by Texas Instruments, then it eliminates the noise in X
# and Y axis and finally it plots the resulting values. It also uses machine learning in order to detect a clap, the
# background color of the graph.

import matplotlib.pyplot as plt
import time
import warnings
import numpy as np
from libs import communications, filterings, graphics, datalog
from sklearn.preprocessing import normalize
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier

warnings.filterwarnings("ignore", category=DeprecationWarning)

communication = communications.CommunicationManager()
filtering = filterings.FilteringManager()
graphic = graphics.GraphicsManager()
report = datalog.DatalogManager()

# Load dataset
train_digits = np.genfromtxt('data/clapping.csv', delimiter = ';')
train_data = train_digits[:, :-1]
train_labels = train_digits[:, -1]

train_data=normalize(train_data)

# Create a classifier
classifier = KNeighborsClassifier()
#classifier = SVC(gamma=0.001, probability=True)
#classifier = DecisionTreeClassifier(random_state=0)
#classifier = RandomForestClassifier(n_estimators = 100)

# Train the classifier
classifier.fit(train_data, train_labels)


class FreeMovementML():
    communication.open_serial_port()
    max_samples = 10
    watch_samples_counter = -1
    save_into_file = True
    lower_index = 0
    higher_index = 30
    snapfingers = 0
    old_prediction=0

    x_axis_acceleration = []
    y_axis_acceleration = []
    z_axis_acceleration = []
    test_digits = []
    time_limit = 60 # Datalog time

    report.create_file('probabilities.txt')
    report.create_file('dataPython.txt')
    report.create_file('clapping.txt')
    report.record_data('dataPython.txt','t', 'x', 'y', 'z')
    k = 0
    t1=0

    time_initial = time.time()
    graphic.set_plot_parameters()

    while time.time() - time_initial <= time_limit:
        time_final = time.time()
        bytes_to_read = communication.send_data_request()
        inbyte = communication.read_data(bytes_to_read)

        if (bytes_to_read == 7 and inbyte[3] == 1) or (bytes_to_read == 14 and inbyte[10] == 1):
            watch_samples_counter += 1

            x_axis_acceleration.append(inbyte[bytes_to_read-3])
            filtering.filter_acceleration(x_axis_acceleration, watch_samples_counter)
            y_axis_acceleration.append(inbyte[bytes_to_read-2])
            filtering.filter_acceleration(y_axis_acceleration, watch_samples_counter)
            z_axis_acceleration.append(inbyte[bytes_to_read-1])
            filtering.filter_acceleration(z_axis_acceleration, watch_samples_counter)
            time_final = time.time()

            report.record_data('dataPython.txt',time_final - time_initial, x_axis_acceleration[watch_samples_counter],
                               y_axis_acceleration[watch_samples_counter], z_axis_acceleration[watch_samples_counter])

            report.record_for_training(x_axis_acceleration[watch_samples_counter],
                                       y_axis_acceleration[watch_samples_counter],
                                       z_axis_acceleration[watch_samples_counter])

            test_digits = test_digits + [x_axis_acceleration[watch_samples_counter],
                                         y_axis_acceleration[watch_samples_counter],
                                         z_axis_acceleration[watch_samples_counter]]

            if watch_samples_counter>=9 and higher_index <= len(test_digits):
                # Load the dataset
                test_data = normalize(test_digits[lower_index:higher_index])

                # Predict values
                test_predicted = classifier.predict(test_data)
                test_probabilities = classifier.predict_proba(test_data)

                report.record_probabilities(int(test_predicted[0]), test_probabilities,test_digits[lower_index:higher_index])

                if test_predicted == 1 and old_prediction==0:
                    snapfingers += 1
                    graphic.change_color()
                    lower_index += 15
                    higher_index += 15
                    print(snapfingers)

                    t1=time.time()
                else:
                    print(time.time()-t1)
                    graphic.restore_color()
                    lower_index += 3
                    higher_index += 3

                old_prediction=test_predicted
            graphic.plot_data(x_axis_acceleration[watch_samples_counter], y_axis_acceleration[watch_samples_counter])

        plt.pause(0.05)  # 50m

        report.next_line()

        k +=1   #communication.close_serial_port()


