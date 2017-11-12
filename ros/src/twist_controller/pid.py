import rospy
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, tune,mn=MIN_NUM, mx=MAX_NUM):
        # self.kp = kp
        # self.ki = ki
        # self.kd = kd
        self.k = [kp, ki, kd]
        if tune:
            self.k = [1, 0, 0]
        self.min = mn
        self.max = mx
        

        self.int_val = self.last_int_val = self.last_error = 0.
        
        # parameters used to tune the pid controller
        self.tune = tune
        self.delta_k = [1, 1, 1]
        self.num_error = 1000
        self.error_sum = 0
        self.min_error_sum = MAX_NUM
        self.selector = 0
        self.operation = ['o', 'o', 'o']
        self.counter = 0

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        
        if self.tune:
            if self.delta_k[0] + self.delta_k[1] + self.delta_k[2] > 0.001:
                self.error_sum += abs(error)
                self.counter += 1
                if self.counter > self.num_error or self.error_sum > self.min_error_sum:
                    self.num_error *= 1.05
                    if self.error_sum < self.min_error_sum:
                        self.min_error_sum = self.error_sum
                        self.delta_k[self.selector] *= 1.1
                        # self.selector = (self.selector + 2) % 3
                        self.k[self.selector] += self.delta_k[self.selector]
                        self.operation[self.selector] = 'i';
                    else:
                        if self.operation[self.selector] == 'i':
                            self.k[self.selector] -= 2 * self.delta_k[self.selector]
                            self.operation[self.selector] = 'd'
                        elif self.operation[self.selector] == 'o':
                            self.k[self.selector] += self.delta_k[self.selector]
                            self.operation[self.selector] = 'i'
                        else:
                            self.k[self.selector] += self.delta_k[self.selector]
                            self.delta_k[self.selector] *= 0.9
                            self.selector = (self.selector + 2) % 3
                            self.k[self.selector] += self.delta_k[self.selector]
                            self.operation[self.selector] = 'i'
                    rospy.loginfo('current solution is, kp: %s, ki: %s, kd: %s',
                    self.k[0], self.k[1], self.k[2])
                    self.counter = 0
                    self.error_sum = 0
            else:
                # best coefficients has been found, save to config file
                rospy.logwarn('solution found, kp: %s, ki: %s, kd: %s', self.k[0], 
                self.k[1], self.k[2])

        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.k[0] * error + self.k[1] * self.int_val + self.k[2] * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
