# pid controller object
# create it and get control data back from it

from __future__ import division

class pid:
    Kp = 1
    Ki = 0
    Kd = 0
    pid_sum = 0
    prior_error = 0
    max_sum = 2

    def __init__(self, Kp_in, Ki_in, Kd_in, max_sum_in = 2):
        self.Kp = Kp_in
        self.Ki = Ki_in
        self.Kd = Kd_in
        self.pid_sum = 0
        self.prior_error = 0
        self.max_sum = max_sum_in

    def __call__(self, e):
        u = self.Kp * e + self.Ki * self.pid_sum + self.Kd * (e - self.prior_error)
        self.prior_error = e
        self.pid_sum += e
        if abs(self.pid_sum) > self.max_sum:
            self.pid_sum /= 2
        return u
    
