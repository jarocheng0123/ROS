import numpy as np
import math

class PidPositional():
    def __init__(self,kp,ki,kd,target,upper=1.0,lower=0.1):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.err=0
        self.err_last=0
        self.err_all=0
        self.target=target
        self.upper=upper
        self.lower=lower
        self.value=0

    def setPid(self,kp,ki,kd):
        self.kp=kp
        self.ki=ki
        self.kd=kd

    def reset(self):
        self.err = 0
        self.err_last = 0
        self.err_all = 0
        self.value=0
    
    def setTarget(self,target):
        self.target = target

    def setLimit(self,upper,lower):
        self.upper=upper
        self.lower=lower


    def calOutput(self,state):
        self.err=self.target-state
        self.value=self.kp*self.err+self.ki*self.err_all+self.kd*(self.err-self.err_last)
        self.err_all=self.err
        self.err_all+=self.err
        flag=1 if self.value>0 else -1
        if abs(self.value) > self.upper:
            self.value = self.upper*flag
        elif abs(self.value) < self.lower:
            self.value = self.lower*flag
        return self.value

class pidIncremental():
    def __init__(self,kp,ki,kd,target,upper=1.0,lower=0.1):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.err=0
        self.err_last=0
        self.err_ll=0
        self.target=target
        self.upper=upper
        self.lower=lower
        self.value=0

    def setPid(self,kp,ki,kd):
        self.kp=kp
        self.ki=ki
        self.kd=kd

    def reset(self):
        self.err = 0
        self.err_last = 0
        self.err_ll = 0
        self.value=0
    
    def setTarget(self,target):
        self.target = target

    def setLimit(self,upper,lower):
        self.upper=upper
        self.lower=lower

    def calOutput(self,state):
        self.err=self.target-state
        self.inc=self.kp*(self.err-self.err_last)+self.ki*self.err+self.kd*(self.err-2*self.err_last+self.err_ll)
        self.err_ll=self.err_last
        self.err_last=self.err
        self.value+=self.inc
        flag=1 if self.value>0 else -1
        if abs(self.value) > self.upper:
            self.value = self.upper*flag
        elif abs(self.value) < self.lower:
            self.value = self.lower*flag
        return self.value