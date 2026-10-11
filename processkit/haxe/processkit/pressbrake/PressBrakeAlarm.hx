package processkit.pressbrake;

/** Named failures retained until a safe, fresh fault-reset request. */
enum PressBrakeAlarm {
	None;
	ConflictingRequests;
	Busy;
	StrokeInterlock;
	OpenInterlock;
	SafetyInterlock;
	InvalidFeedback;
	ProgramChanged;
	RegistrationLost;
	RobotHoldLost;
	PositioningLost;
	ClampTimeout;
	WithdrawalTimeout;
	DepthTimeout;
	DecompressionTimeout;
	OpenTimeout;
}
