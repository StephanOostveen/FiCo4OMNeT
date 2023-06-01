enum TaskState {
    Running = 0; 
    Ready = 1;
    Paused = 2;
    Blocked = 3;
};

message SchedulerEvent {
    TaskState state;
};