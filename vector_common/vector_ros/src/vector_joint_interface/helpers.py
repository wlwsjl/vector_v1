import rospy
import math
import threading

def limit(signal_in,signal_limit):

    if (signal_in >= signal_limit):
        ret = signal_limit 
    elif (signal_in <= -signal_limit):
        ret = -signal_limit
    else:
        ret = signal_in
    return ret

                
class RateLimitSignals(object):
    def __init__(self,max_rate,num_sigs=1,sig_init=[0.0]):
        self._max_rate = max_rate
        self._sigout = sig_init
        self._numsigs = num_sigs
        self._prev_time = rospy.get_time()
    
    def SetMaxRate(self,rate):
        self._max_rate = rate
    def Reset(self,sig_init=[0.0]):
        self._sigout = sig_init
        self._prev_time = rospy.get_time()
    def Update(self,sigin):
        now = rospy.get_time()
        dt = now - self._prev_time
        self._prev_time = now
        if (dt > 0.0):
            for i in range(self._numsigs):
                requested_rate = (sigin[i] - self._sigout[i])/dt
                            
                if (requested_rate > self._max_rate[i]):
                    self._sigout[i] += self._max_rate[i] * dt
                elif (requested_rate < -self._max_rate[i]):
                    self._sigout[i] += -self._max_rate[i] * dt
                else:
                    self._sigout[i] = sigin[i]
        return self._sigout

class DifferentiateSignals(object):
    def __init__(self,num_sigs=1,sig_init=[0.0]):
        self._prev_sigin = sig_init
        self._numsigs = num_sigs
        self._prev_time = rospy.get_time()
    def Reset(self,sig_init=[0.0]):
        self._prev_sigin = sig_init
        self._prev_time = rospy.get_time()
    def Update(self,sigin):
        now = rospy.get_time()
        dt = now - self._prev_time
        self._prev_time = now
        ret = [0.0]*self._numsigs
        if (dt > 0.0):
            ret = [(sigin[i]-self._prev_sigin[i])/dt for i in range(self._numsigs)]
        self._prev_sigin = sigin 
        return ret
      

class FilterSignals(object):
    def __init__(self,cutoff_freq,num_sigs=1,sig_init=[0.0]):
        self._cutoff_freq = cutoff_freq
        self._sigout = sig_init
        self._numsigs = num_sigs
        self._prev_time = rospy.get_time()
        
    def SetCuttoffFreq(self,cutoff_freq):
        self._cutoff_freq = cutoff_freq
    def Reset(self,sig_init=[0.0]):
        self._sigout = sig_init
        self._prev_time = rospy.get_time()
    def Update(self,sigin):
        now = rospy.get_time()
        dt = now - self._prev_time
        self._prev_time = now
        if (dt > 0.0):
            filter_const = math.exp((-1.0 * (self._cutoff_freq * (2.0 * math.pi))) * dt)
            self._sigout = [sigin[i] + (filter_const * (self._sigout[i] - sigin[i])) for i in range(self._numsigs)]
        return self._sigout


