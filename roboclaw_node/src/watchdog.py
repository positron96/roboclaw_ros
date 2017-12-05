class Watchdog:
	_nextTime = None
	_secs = None
	_fn = None
	def __init__(self, secs, fn):
		self._secs = secs
		self._fn = fn		
		
	def kick(self):
		self._nextTime = time.time() + self._secs
		
	def check(self):
		if( time.time() > self._nextTime):
			self._fn()
			self.kick()
		
