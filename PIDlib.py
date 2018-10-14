class PID(object): #Run me on a loop
	def __init__(self):		
		#Variables
		self.Kp=1
		self.Ki=0
		self.Kd=0
		
		self.SampleTime=1e-3 #Default settings expect seconds.
		self.clock=0 #Clock variable
	
		self.OnOffSw=0; #PID is OFF by default
		self.error=0;
		self.ITerm=0;
		self.lastC=0; #Time last cycle
		self.lastVal=0; #Last current value
		
		#PID output limits
		self.uplim=2000;
		self.dwnlim=0;
				
	#Functions
	def ComputePID(self,cval,setpt):
		out=0
		if(self.OnOffSw!=1):
			return out
		#Check sample time
		if self.SampleTime<=self.clock-self.lastC:
			self.lastC=self.clock
			
			#Calculate error
			self.error=setpt-cval
			self.ITerm+=self.Ki*self.error
			
			#Limit ITerm
			if self.ITerm<self.dwnlim:
				out=self.dwnlim
			elif self.ITerm>self.uplim:
				out=self.uplim
			
			#Calculate output of the controller
			out=self.Kp * self.error + self.ITerm - self.Kd * (cval-self.lastVal)
			
			self.lastVal=cval
			
			#Limit the controller
			if out<self.dwnlim:
				out=self.dwnlim
			elif out>self.uplim:
				out=self.uplim
			
			return out
					
	def SetTunings(self,kp,ki,kd):
		self.Kp=kp;
		self.Ki=ki*self.SampleTime;
		self.Kd=kd/self.SampleTime;
		
	def SetSampleTime(self,NewSampleTime):
		if NewSampleTime>0:
			ratio=NewSampleTime/self.SampleTime
			self.Ki*=ratio
			self.Kd/=ratio
			self.SampleTime=NewSampleTime
		
	def StopPID(self):
		self.OnOffSw=0
		
	def StartPID(self):
		self.OnOffSw=1
		
	def Reset(self): #Resets PID private variables
		self.error=0
		self.lastVal=0
		self.ITerm=0
