import time


class pid:
    def __init__(self,Kp,Ki,Kd,offset=0):
        self.P = 0
        self.I = 0
        self.D = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Offset = offset
        self.last_time = 0
        self.time = 0
        self.last_Error = 0
        self.Error = 0
        self.last_pid = 0
        
        self.pid_limit = None 
        self.I_limit = None 
        
    def update_pid(self,point,data):
        self.Error = -(point - data)
        self.time = time.time()

        elapsed_time = int((self.time - self.last_time)*1000)
        
        if elapsed_time:

            self.P = self.Error * self.Kp
            self.D = ((self.Error - self.last_Error)/(elapsed_time) )* self.Kd
            self.I += (self.Error ) * self.Ki

            if self.I_limit:
                if self.I > self.I_limit: self.I = self.I_limit
                if self.I < -self.I_limit: self.I = -self.I_limit

            self.last_time = self.time
            self.last_Error = self.Error
            pid = (self.P + self.I + self.D )

            if self.pid_limit:
                if pid > self.pid_limit: pid = self.pid_limit
                if pid < -self.pid_limit: pid = -self.pid_limit
            self.last_pid = pid
            return pid + self.Offset
        else:
            return self.last_pid + self.Offset

    def get_term_i(self):
        return self.I
    def set_term_ki(self,Ki):
        self.Ki = Ki

    def get_term_p(self):
        return self.P
    def set_term_kp(self,Kp):
        self.Kp = Kp

    def get_term_d(self):
        return self.D
    def set_term_kd(self,Kd):
        self.Kd = Kd

    def set_term_offset(self,offset):
        self.Offset = offset

    def set_pid_limit(self,limit):
        self.pid_limit = limit
        
        
    def set_I_limit(self,I_limit):
        self.I_limit = I_limit

    def get_error(self):
        return self.Error
        
    def resetI(self):
         self.I = 0
