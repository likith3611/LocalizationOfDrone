#!/usr/bin/env python
from pid_tune.msg import PidTune
import rospy
import time
from geometry_msgs.msg import Point



class Tune():
    def __init__(self, pid_effort_topic, max, min, step, threshold_time, index):
        rospy.init_node('tune_pid')
        # Publisher to give output PID values
        self.send_pid = rospy.Publisher(pid_effort_topic, PidTune, latch = True, queue_size = 10)
        self.pid_value = PidTune()
        self.set = Point()
        self.state = Point()
        self.rate = rospy.Rate(50)
        self.pid_tuned = False
        self.thresh = threshold_time
        self.index = index
        rospy.Subscriber('/statespecs/pose', Point, self.get_statepose)
        rospy.Subscriber('/setespecs/pose', Point, self.get_setpose)


        # Define how rapidly the value of Kp is variedm and the max/min values to try
        self.Kp_min = min
        self.Kp_max = max
        self.Kp_step = step

        # Variable to count number of oscillations
        self.oscillation_count = 0

    def tune_pid (self):
        kp = self.Kp_min
        self.setKiKdToZero()

        while (kp<self.Kp_max and (self.pid_tuned is not True)):
            #print self.pid_tuned
            self.pid_value.Kp = kp
            self.set_value()
            self.auto_ZN(kp)
            kp = kp + self.Kp_step
            if (self.pid_tuned == True):
                break
        if (not self.pid_tuned):
            print("Did not see any oscillations for this range of Kp. Adjust "
                    "Kp_max and Kp_min to broaden the search.")
        # rospy.signal_shutdown("Coudn't find the values")


    def set_value(self):
        #print "publishing", self.pid_value.Kp
        self.send_pid.publish(self.pid_value)

    def setKiKdToZero (self):
        self.pid_value.Ki = 0
        self.pid_value.Kd = 0

    def auto_ZN (self, start_kp):
        self.pid_value.Kp = start_kp #Set minimum value of Kp
        print"Trying Kp value = ", self.pid_value.Kp

        for i in range (100):
            # rospy.spin()
            # self.rate.sleep()

            rospy.wait_for_message("statespecs/pose", Point)
            #print "here"

            if (i == 0):
                initial_error = []
                #print self.set.x, self.state.x
                initial_error.append(self.set.x - self.state.x)    # Get the sign of the initial error
                initial_error.append(self.set.y - self.state.y)    # Get the sign of the initial error
                initial_error.append(-18 - self.state.z)    # Get the sign of the initial error

               # Did the system oscillate about the setpoint? If so, Kp~Ku.
               # Oscillation => the sign of the error changes
               # The first oscillation is caused by the change in setpoint. Ignore it.
               # Look for 2 oscillations.
               # Get a fresh state message
            rospy.wait_for_message("statespecs/pose", Point)
            new_error = []
            new_error.append(self.set.x - self.state.x)    #Sign of the new error
            new_error.append(self.set.y - self.state.y)    #Sign of the new error
            new_error.append(-18 - self.state.z)   #Sign of the new error

            # print initial_error[self.index], new_error[self.index]
            if ((1 if initial_error[self.index] < 0 else 0) != (1 if new_error[self.index] < 0 else 0)):    # To check if the sign has changed or not
                 self.oscillation_count = self.oscillation_count + 1
                 self.x=time.time()
                 initial_error[self.index] = new_error[self.index]    # Reset to look for next oscillations
                 print "Oscillation occurred. Oscillation count:  " , self.oscillation_count

            # if the system is definitely oscillating
            if (self.oscillation_count > 2):
                #now end  calculating the time
                time1 = time.time()-self.x
                # time1 = self.x



            # We're looking for more than just the briefest dip across the
            # setpoint and back.
            # Want to see significant oscillation
                if (time1 > self.thresh):
                    Ku = self.pid_value.Kp

                    #Now Calculating other parameters
                    self.pid_value.Kp = int(0.6 * Ku)
                    self.pid_value.Ki = int(2 * Ku / time1)
                    self.pid_value.Kd = int(Ku * time1 / 3)

                    print("Values for PID constants found: ", self.pid_value.Kp, self.pid_value.Ki, self.pid_value.Kd)
                    self.set_value()
                    # rospy.signal_shutdown("PID Tuning Complete")
                    self.pid_tuned = True
                    break



    def get_setpose(self, msg):
        self.set = msg

    def get_statepose(self, msg):
        self.state = msg


if __name__ == '__main__':
    throt_pid_tune = Tune("/pid_tuning_altitude", 5000, 1200, 200, 2, 1)
    pitch_pid_tune = Tune("/pid_tuning_pitch", 5000, 2000, 100, 1.5, 2)
    roll_pid_tune = Tune("/pid_tuning_roll", 5000, 500, 100, .5, 0)
    while not rospy.is_shutdown():
        throt_pid_tune.tune_pid()
        roll_pid_tune.tune_pid()
        # pitch_pid_tune.tune_pid()
        exit()
        # rospy.spin()
